import numpy as np
import tensorflow as tf
from tensorflow.keras import Sequential, layers, losses
from tensorflow.keras.models import Model
from tensorflow.keras.callbacks import ModelCheckpoint
from tensorflow.data import Dataset
import sys
import os
from datetime import datetime
import matplotlib.pyplot as plt
import gc

BATCH_SIZE = 32
EPOCHS = 200
MODE = 'LSTM'

class LSTMAutoencoder(Model):
    """docstring for LSTMAutoencoder"""
    def __init__(self, latent_dim = 8,window = 30, input_size = 26, enc_layers = [], dec_layers = []):
        super(LSTMAutoencoder, self).__init__()
        self.latent_dim = latent_dim
        self.input_size = input_size
        self.encoder = Sequential(
            [layers.LSTM(size, activation = 'relu',return_sequences = True) for size in enc_layers] + 
            [layers.LSTM(self.latent_dim, activation = 'relu',return_sequences = True)])
        self.decoder = Sequential(
            [layers.LSTM(size, activation = 'relu',return_sequences = True) for size in dec_layers] + 
            [layers.LSTM(self.input_size, activation = 'tanh')])

    def call(self, x):
        return self.decoder(self.encoder(x))

class Autoencoder(Model):
    """docstring for Autoencoder"""
    def __init__(self, latent_dim = 8, input_size = 26, enc_layers = [], dec_layers = []):
        super(Autoencoder, self).__init__()
        self.latent_dim = latent_dim
        self.input_size = input_size
        self.encoder = Sequential(
            [layers.Input(shape = (self.input_size))] + 
            [layers.Dense(size, activation = 'relu') for size in enc_layers] + 
            [layers.Dense(self.latent_dim, activation = 'relu')])
        self.decoder = Sequential(
            [layers.Input(shape = (self.latent_dim))] + 
            [layers.Dense(size, activation = 'relu') for size in dec_layers] + 
            [layers.Dense(self.input_size, activation = 'tanh')])
        self.encoder.summary()
        self.decoder.summary()

    def call(self, x):
        return self.decoder(self.encoder(x))

def main():
    train_ds = None
    for ds_file in os.listdir(sys.argv[1]):
        print(f'Loading file: {ds_file}')
        new_ds = Dataset.load(sys.argv[1] + '/' + ds_file)
        if train_ds == None:
            train_ds = new_ds
        else:
            train_ds = train_ds.concatenate(new_ds)
        print(f'New dataset size: {len(list(train_ds))}')
        del new_ds
        gc.collect()
        # break


    # remove empty angle values from imu
    train_ds = train_ds.map(lambda x: tf.concat([x[:20], x[21:]], 0))

    # calculate absolute maximum for each column
    maximum = np.absolute(np.array(list(train_ds))).max(axis = 0)
    print(f'Absolute maximum: {maximum}')

    # prepare dataset - normalize, group,batch
    train_ds = train_ds.map(lambda x: x/maximum)

    if MODE == 'AE':
        train_ds = train_ds.map(lambda x: (x, x))
        train_ds = train_ds.batch(BATCH_SIZE).prefetch(1)
        for LATENT_DIM in range(6, 14):
            for ENC_LAYERS in [[], [16], [32, 16], [64, 32, 16]]:
                # LATENT_DIM = 8
                # ENC_LAYERS = []
                DEC_LAYERS = ENC_LAYERS[::-1]
                output_folder = datetime.now().strftime(f'odometry_anomaly_detector/models/autoencoder/{len(list(train_ds))}_E{EPOCHS}_B{BATCH_SIZE}_L{LATENT_DIM}_E{ENC_LAYERS}_D{DEC_LAYERS}_%Y%m%d_%H%M%S/')
                autoencoder = Autoencoder(latent_dim = LATENT_DIM, enc_layers = ENC_LAYERS, dec_layers = DEC_LAYERS)
                autoencoder.compile(optimizer = 'adam', loss = losses.MeanSquaredError())
                history = autoencoder.fit(train_ds, batch_size = BATCH_SIZE, epochs = EPOCHS, shuffle = True, callbacks = [ModelCheckpoint(output_folder + 'save_{epoch}', save_weights_only = True)])

                plt.figure()
                plt.plot(history.history['loss'], label = 'loss')
                plt.title(f'Loss after every epoch (lowest loss: {min(history.history["loss"]):.5f})')
                # os.makedirs(output_folder)
                plt.savefig(output_folder + 'history.jpg')
                # plt.show()
                del autoencoder
                del history
                gc.collect()

    elif MODE == 'LSTM':
        for WINDOW_SIZE in [30]: # [2, 5, 10, 30]:
            lstm_train_ds = train_ds.window(WINDOW_SIZE, shift = 1, drop_remainder = True)
            # print(list(lstm_train_ds)[-1])
            lstm_train_ds = lstm_train_ds.flat_map(lambda window: window.batch(WINDOW_SIZE))
            # print(list(lstm_train_ds)[-1])
            lstm_train_ds = lstm_train_ds.map(lambda window: (window, window[-1]))
            # print(list(lstm_train_ds)[-1])
            lstm_train_ds = lstm_train_ds.batch(BATCH_SIZE).prefetch(1)
            # print(list(lstm_train_ds)[-1])
            for LATENT_DIM in [11]: #range(6, 14):
                for ENC_LAYERS in [[16], [32, 16], [64, 32, 16]]:#[[], [16], [32, 16], [64, 32, 16]]:
                    # LATENT_DIM = 8
                    # ENC_LAYERS = []
                    DEC_LAYERS = ENC_LAYERS[::-1]
                    output_folder = datetime.now().strftime(f'odometry_anomaly_detector/models/lstmautoencoder/{len(list(lstm_train_ds))}_E{EPOCHS}_W{WINDOW_SIZE}_B{BATCH_SIZE}_L{LATENT_DIM}_E{ENC_LAYERS}_D{DEC_LAYERS}_%Y%m%d_%H%M%S/')
                    autoencoder = LSTMAutoencoder(latent_dim = LATENT_DIM, window = WINDOW_SIZE, enc_layers = ENC_LAYERS, dec_layers = DEC_LAYERS)
                    autoencoder.compile(optimizer = 'adam', loss = losses.MeanSquaredError())
                    history = autoencoder.fit(lstm_train_ds, epochs = EPOCHS, callbacks = [ModelCheckpoint(output_folder + 'save_{epoch}', save_weights_only = True)])

                    plt.figure()
                    plt.plot(history.history['loss'], label = 'loss')
                    plt.title(f'Loss after every epoch (lowest loss: {min(history.history["loss"]):.5f})')
                    # os.makedirs(output_folder)
                    plt.savefig(output_folder + 'history.jpg')
                    # plt.show()
                    del autoencoder
                    del history
                    gc.collect()

            del lstm_train_ds
            gc.collect()

        del train_ds
        gc.collect()

if __name__ == '__main__':
    main()