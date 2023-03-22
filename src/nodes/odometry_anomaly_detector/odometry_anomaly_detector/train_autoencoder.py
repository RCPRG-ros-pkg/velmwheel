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

BATCH_SIZE = 32
LATENT_DIM = 8
EPOCHS = 100

class Autoencoder(Model):
    """docstring for Autoencoder"""
    def __init__(self, latent_dim = 8, input_size = 27):
        super(Autoencoder, self).__init__()
        self.latent_dim = latent_dim
        self.input_size = input_size
        self.encoder = Sequential([
            # layers.Dense(32, activation = 'relu')
            layers.Dense(self.latent_dim, activation = 'relu')
        ])
        self.decoder = Sequential([
            layers.Dense(self.input_size, activation = 'relu'),
            layers.Dense(self.input_size, activation = 'linear')
        ])

    def call(self, x):
        return self.decoder(self.encoder(x))

def main():
    train_ds = None
    for ds_file in os.listdir(sys.argv[1]):
        new_ds = Dataset.load(sys.argv[1] + '/' + ds_file)
        if train_ds == None:
            train_ds = new_ds
        else:
            train_ds = train_ds.concatenate(new_ds)
        print(ds_file)
        print(len(list(train_ds)))
        print(list(train_ds)[-1])
    output_folder = datetime.now().strftime(f'odometry_anomaly_detector/models/{len(list(train_ds))}_E{EPOCHS}_B{BATCH_SIZE}_L{LATENT_DIM}_%Y%m%d_%H%M%S_%f/')
    train_ds = train_ds.map(lambda x: (x, x))
    train_ds = train_ds.batch(BATCH_SIZE).prefetch(1)

    autoencoder = Autoencoder(latent_dim = LATENT_DIM)
    autoencoder.compile(optimizer = 'adam', loss = losses.MeanSquaredError())
    history = autoencoder.fit(train_ds, batch_size = BATCH_SIZE, epochs = EPOCHS, shuffle = True, callbacks = [ModelCheckpoint(output_folder + 'save_{epoch}', save_weights_only = True)])

    plt.figure()
    plt.plot(history.history['loss'], label = 'loss')
    plt.title('Loss after every epoch')
    plt.savefig(output_folder + 'history.jpg')
    plt.show()

if __name__ == '__main__':
    main()