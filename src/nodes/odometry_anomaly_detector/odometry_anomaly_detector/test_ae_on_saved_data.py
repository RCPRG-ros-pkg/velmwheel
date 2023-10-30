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
# first
# MAXIMUM = np.array([float(x) for x in ('2.0636522e+02 9.3049286e+01 1.7743228e+02 7.7254242e+01 2.2515166e+00 2.1266069e+00 2.1282153e+00 2.0365186e+00 1.4761110e-01 1.2193915e-01 3.1414101e+00 1.9489980e-01 1.1072106e+00 3.5551035e-01 3.1414123e+00 5.0564921e-01 1.4996742e+00 5.5352163e+00 7.1549611e+00 1.6677181e-01 8.3385907e-02 8.3385907e-02').split(' ')])
# THRESHOLD = 0.051682
# THRESHOLD2 = 0.101741
# front_back
# MAXIMUM = np.array([float(x) for x in ('11.081729 28.966188 28.886015 11.037709 0.969395 0.96281016 0.9952817 0.8621033 0.086427286 0.046184205 0.002854337 0.0633505 1.1006445 0.14612588 0.049016092 0.11947878 0.7360926 1.9005876 6.5704165 1.0 0.08338591 1.0').split(' ')])
# THRESHOLD = 0.021734
# THRESHOLD2 = 0.021734
# giant
# MAXIMUM = np.array([float(x) for x in ('192.96597 141.35614 284.50732 113.91022 2.8158975 2.9315457 2.713783 2.693702 0.13637446 0.15022385 3.1414652 0.23251095 1.1199563 0.46944243 3.1414676 0.34841314 1.7606009 5.002921 8.242411 0.1667646 0.08338314 0.083378814').split(' ')])
# THRESHOLD = 0.029547
# THRESHOLD2 = 0.031914
# half milion
MAXIMUM = np.array([float(x) for x in ('192.96597 141.35614 284.50732 113.91022 2.853408 2.9315457 2.7728324 2.7082665 0.14604822 0.1565431 3.1414652 0.23251095 1.4040922 0.58406204 3.1415603 0.39687586 1.7606009 5.4503107 8.242411 0.16676718 0.08338314 0.083378814').split(' ')])
THRESHOLD = 0.019916
THRESHOLD2 = 0.024037


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
        del new_ds
        gc.collect()
        # break

    ds_size = len(list(test_ds))
    print(f'New dataset size: {ds_size}')
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
    plt.plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), test_loss, label = 'Reconstruction error')
    plt.axhline(y=THRESHOLD, color='r', linestyle='-',label = 'Anomaly threshold')
    # plt.axhline(y=THRESHOLD2, color='m', linestyle='-',label = 'Anomaly threshold')
    plt.title('Reconstruction error in time')
    plt.ylabel('Error')
    plt.xlabel('Time [s]')
    plt.legend()
    plt.show()

    # fig,ax = plt.subplots(3,1)
    # # ax = fig.add_subplot(3,1,1)
    # ax[0].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,19])
    # ax[0].set_ylabel('Rotation')
    # ax[1].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,20])
    # ax[1].set_ylabel('Forward\nvelocity')
    # ax[2].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,21])
    # ax[2].set_ylabel('Sideways\nvelocity')
    # ax[2].set_xlabel('Time [s]')
    # plt.show()

    # fig,ax = plt.subplots(3,1)
    # # ax = fig.add_subplot(3,1,1)
    # ax[0].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,16])
    # ax[0].set_ylabel('Angular velocity')
    # ax[1].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,17])
    # ax[1].set_ylabel('Linear\nacceleration X')
    # ax[2].plot(np.linspace(0, (ds_size - 1)/30, num = ds_size), np.array(list(test_ds))[:,:,-1,18])
    # ax[2].set_ylabel('Linear\nacceleration Y')
    # ax[2].set_xlabel('Time [s]')
    # plt.show()

if __name__ == '__main__':
    main()