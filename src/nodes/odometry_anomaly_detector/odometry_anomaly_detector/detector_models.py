from tensorflow.keras import Sequential, layers
from tensorflow.keras.models import Model

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