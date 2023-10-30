import numpy as np
import tensorflow as tf
from tensorflow.keras import losses
from tensorflow.keras.callbacks import ModelCheckpoint
from tensorflow.data import Dataset
import sys
import os
from datetime import datetime
import matplotlib.pyplot as plt
import gc
from odometry_anomaly_detector.detector_models import Autoencoder, LSTMAutoencoder
from copy import copy

BATCH_SIZE = 32
EPOCHS = 200
MODE = 'LSTM'
LATENT_DIM = 11
ENC_LAYERS = []
DEC_LAYERS = ENC_LAYERS[::-1]
WINDOW_SIZE = 10
SPLIT = 0.9

def main():
    train_ds = None
    test_ds = None
    maximum = np.empty(0)
    for ds_file in os.listdir(sys.argv[1]):
        print(f'Loading file: {ds_file}')
        new_ds = Dataset.load(sys.argv[1] + '/' + ds_file)
        # remove empty angle values from imu and positions from odometry
        new_ds = new_ds.map(lambda x: tf.concat([x[0:8], x[10:14], x[16:20], x[21:]], 0))
        # recalculate maximum
        if maximum.size > 0:
            new_maximum = np.absolute(np.array(list(new_ds))).max(axis = 0)
            maximum = np.array([maximum, new_maximum]).max(axis = 0)
            del new_maximum
        else:
            maximum = np.absolute(np.array(list(new_ds))).max(axis = 0)
        # sort into windows
        if WINDOW_SIZE > 1:
            new_ds = new_ds.window(WINDOW_SIZE, shift = 1, drop_remainder = True)
            new_ds = new_ds.flat_map(lambda window: window.batch(WINDOW_SIZE))
        # add to full dataset
        if train_ds == None:
            train_ds = new_ds
        else:
            train_ds = train_ds.concatenate(new_ds)
        del new_ds
        gc.collect()
        # break

    ds_size = len(list(train_ds))
    print(f'New dataset size: {ds_size}')
    # divide by maximum for each column
    print(f'Absolute maximum: {maximum}')
    for i,m in enumerate(maximum):
        if m == 0:
            maximum[i] = 1.0
    train_ds = train_ds.map(lambda window: window/maximum)

    # divide into train and test dataset
    print(f'Shuffling dataset...')
    train_ds.shuffle(ds_size)
    # test_ds = copy(train_ds)
    test_ds = train_ds.skip(int(SPLIT * ds_size))
    train_ds = train_ds.take(int(SPLIT * ds_size))

    print(f'Train ds: {len(list(train_ds))}\tTest_ds: {len(list(test_ds))}')

    # input and output
    if WINDOW_SIZE > 1:
        train_ds = train_ds.map(lambda window: (window, window[-1]))
    else:
        train_ds = train_ds.map(lambda window: (window, window))

    if MODE == 'AE':
        output_folder = datetime.now().strftime(f'odometry_anomaly_detector/models/autoencoder/{len(list(train_ds))}_{len(list(test_ds))}_E{EPOCHS}_B{BATCH_SIZE}_L{LATENT_DIM}_E{str(ENC_LAYERS)[1:-1]}_D{str(DEC_LAYERS)[1:-1]}_%Y%m%d_%H%M%S/')
        autoencoder = Autoencoder(latent_dim = LATENT_DIM, enc_layers = ENC_LAYERS, dec_layers = DEC_LAYERS, input_size = 22)

    elif MODE == 'LSTM':
        output_folder = datetime.now().strftime(f'odometry_anomaly_detector/models/lstmautoencoder/{len(list(train_ds))}_{len(list(test_ds))}_E{EPOCHS}_W{WINDOW_SIZE}_B{BATCH_SIZE}_L{LATENT_DIM}_E{str(ENC_LAYERS)[1:-1]}_D{str(DEC_LAYERS)[1:-1]}_%Y%m%d_%H%M%S/')
        autoencoder = LSTMAutoencoder(latent_dim = LATENT_DIM, window = WINDOW_SIZE, enc_layers = ENC_LAYERS, dec_layers = DEC_LAYERS, input_size = 22)

    try:
        os.makedirs(output_folder)
    except:
        pass

    with open(output_folder + 'maximum.txt', 'w') as f:
        f.write(' '.join([str(m) for m in maximum]))

    train_ds.save(output_folder + 'train_ds')
    test_ds.save(output_folder + 'test_ds')

    train_ds = train_ds.batch(BATCH_SIZE).prefetch(1)
    # test_ds_pre_batch = test_ds
    test_ds = test_ds.batch(1)
    
    autoencoder.compile(optimizer = 'adam', loss = losses.MeanSquaredError())
    # history = autoencoder.fit(train_ds, epochs = EPOCHS)
    history = autoencoder.fit(train_ds, epochs = EPOCHS, callbacks = [ModelCheckpoint(output_folder + 'save_{epoch}', save_weights_only = True)])

    plt.figure()
    plt.plot(history.history['loss'], label = 'loss')
    plt.title(f'Loss after every epoch (lowest loss: {min(history.history["loss"]):.5f})')
    plt.xlabel('Epoch')
    plt.ylabel('Training loss')
    plt.savefig(output_folder + 'history.eps')
    # plt.show()
    del history
    gc.collect()

    reconstructions = np.array([autoencoder(x) for x in test_ds]).squeeze()
    # print(reconstructions.shape)
    test_loss = tf.keras.losses.mae(reconstructions, [window[-1] for window in test_ds.unbatch()])

    threshold = np.mean(test_loss) + np.std(test_loss)
    print(f'Estimated anomaly threshold:{threshold}')

    plt.figure()
    plt.hist(test_loss[None, :], bins=50)
    plt.title(f'MAE of test samples, anomaly threashold: {threshold}, mean:{np.mean(test_loss)}, std dev:{np.std(test_loss)}')
    plt.xlabel("Test loss")
    plt.ylabel("No of examples")
    plt.savefig(output_folder + 'test_samples.eps')
    # plt.show()

    del autoencoder
    del train_ds
    del test_ds
    gc.collect()

if __name__ == '__main__':
    main()