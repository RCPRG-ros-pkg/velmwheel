import numpy as np
import tensorflow as tf
from tensorflow.keras import losses
from tensorflow.data import Dataset
import sys
import os
from datetime import datetime
import matplotlib.pyplot as plt
import gc
from odometry_anomaly_detector.detector_models import LSTMAutoencoder

EPOCHS = 200
MODE = 'LSTM'
LATENT_DIM = 11
ENC_LAYERS = []
DEC_LAYERS = ENC_LAYERS[::-1]
WINDOW_SIZE = 10
# MAXIMUM = np.array([float(x) for x in ('2.0636522e+02 9.3049286e+01 1.7743228e+02 7.7254242e+01 2.2515166e+00 2.1266069e+00 2.1282153e+00 2.0365186e+00 1.4761110e-01 1.2193915e-01 3.1414101e+00 1.9489980e-01 1.1072106e+00 3.5551035e-01 3.1414123e+00 5.0564921e-01 1.4996742e+00 5.5352163e+00 7.1549611e+00 1.6677181e-01 8.3385907e-02 8.3385907e-02').split(' ')])
MAXIMUM = np.array([float(x) for x in ('10.934464 23.608767 33.97022 11.812075 1.270523 1.6523898 1.676781 1.2256358 0.08889163 0.08581814 1.6261551 0.16901387 0.8425416 0.2769558 1.601686 0.30849126 1.0642145 3.0010996 3.471348 0.16677181 0.08338591 0.08338591').split(' ')])
# THRESHOLD = 0.051682
# THRESHOLD2 = 0.101741
THRESHOLD = 0.038426
THRESHOLD2 = 0.038426

def main():
    test_ds = None

    # Load ds
    ds_files = os.listdir(sys.argv[2])
    ds_files.sort()
    for ds_file in ds_files:
        print(f'Loading file: {ds_file}')
        new_ds = Dataset.load(sys.argv[2] + '/' + ds_file)
        # remove empty angle values from imu and positions from odometry
        new_ds = new_ds.map(lambda x: tf.concat([x[0:8], x[10:14], x[16:20], x[21:]], 0))
        # sort into windows
        if WINDOW_SIZE > 1:
            new_ds = new_ds.window(WINDOW_SIZE, shift = 1, drop_remainder = True)
            new_ds = new_ds.flat_map(lambda window: window.batch(WINDOW_SIZE))
        # add to full dataset
        if test_ds == None:
            test_ds = new_ds
        else:
            test_ds = test_ds.concatenate(new_ds)
        ds_size = len(list(test_ds))
        print(f'New dataset size: {ds_size}')
        del new_ds
        gc.collect()
        # break

    # divide by maximum for each column and batch
    test_ds = test_ds.map(lambda window: window/MAXIMUM)
    test_ds = test_ds.batch(1)

    # test_ds = Dataset.load(sys.argv[2])
    # ds_size = len(list(test_ds))
    # print(f'New dataset size: {ds_size}')
    # test_ds = test_ds.batch(1)

    if MODE == 'AE':
        autoencoder = Autoencoder(latent_dim = LATENT_DIM, enc_layers = ENC_LAYERS, dec_layers = DEC_LAYERS, input_size = 22)

    elif MODE == 'LSTM':
        autoencoder = LSTMAutoencoder(latent_dim = LATENT_DIM, window = WINDOW_SIZE, enc_layers = ENC_LAYERS, dec_layers = DEC_LAYERS, input_size = 22)

    autoencoder.compile(optimizer = 'adam', loss = losses.MeanSquaredError())
    autoencoder.load_weights(sys.argv[1])
    print(f'Weights loaded.')

    # for x,i in enumerate(test_ds):
        # print(f'Reconstruction: {autoencoder(x)}\nOriginal: {x[:,-1]}')
        # break
    # return

    # print(np.array(list(test_ds)).shape)
    # return

    # reconstruct
    reconstructions = np.array([autoencoder(x) for x in test_ds]).squeeze()
    test_loss = tf.keras.losses.mae(reconstructions, [window[-1] for window in test_ds.unbatch()])
    threshold = np.mean(test_loss) + np.std(test_loss)
    print(f'Proposed threshold: {threshold}')

    plt.figure()
    plt.plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), test_loss)
    plt.axhline(y=THRESHOLD, color='r', linestyle='-')
    plt.axhline(y=THRESHOLD2, color='m', linestyle='-')
    plt.title('Reconstruction error in time')
    plt.ylabel('Error')
    plt.xlabel('Time [s]')
    plt.show()

    fig,ax = plt.subplots(3,1)
    # ax = fig.add_subplot(3,1,1)
    ax[0].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,19])
    ax[0].set_ylabel('Rotation')
    ax[1].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,20])
    ax[1].set_ylabel('Forward\nvelocity')
    ax[2].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,21])
    ax[2].set_ylabel('Sideways\nvelocity')
    ax[2].set_xlabel('Time [s]')
    plt.show()

    fig,ax = plt.subplots(3,1)
    # ax = fig.add_subplot(3,1,1)
    ax[0].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,16])
    ax[0].set_ylabel('Angular velocity')
    ax[1].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,17])
    ax[1].set_ylabel('Linear\nacceleration X')
    ax[2].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,18])
    ax[2].set_ylabel('Linear\nacceleration Y')
    ax[2].set_xlabel('Time [s]')
    plt.show()

if __name__ == '__main__':
    main()