#!/usr/bin/python3
import sys
import os
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.data import Dataset
import gc
from datetime import datetime

BATCH_SIZE = 32
EPOCHS = 100

def make_generator_model(input_size = 1, output_size = 1):
	model = tf.keras.Sequential()

	model.add(layers.Dense(32, use_bias = False, input_shape = (input_size,)))
	model.add(layers.LeakyReLU())
	
	model.add(layers.Dense(64, use_bias = False))
	model.add(layers.LeakyReLU())
	
	model.add(layers.Dense(128, use_bias = False))
	model.add(layers.LeakyReLU())
	
	model.add(layers.Dense(64, use_bias = False))
	model.add(layers.LeakyReLU())
	
	model.add(layers.Dense(32, use_bias = False))
	model.add(layers.LeakyReLU())

	model.add(layers.Dense(output_size, use_bias = False))
	model.add(layers.Activation('tanh'))
	assert model.output_shape == (None, output_size)
	
	return model

def make_discriminator_model(input_size = 1):
	model = tf.keras.Sequential()
	
	model.add(layers.Dense(128, use_bias = False, input_shape = (input_size,)))
	model.add(layers.LeakyReLU())
	
	model.add(layers.Dense(64, use_bias = False))
	model.add(layers.LeakyReLU())

	model.add(layers.Dense(1, use_bias = False))
	model.add(layers.Activation('sigmoid'))
	assert model.output_shape == (None, 1)
	
	return model

def discriminator_loss(real_output, fake_output):
	cross_entropy = tf.keras.losses.BinaryCrossentropy(from_logits=False)
	real_loss = cross_entropy(tf.ones_like(real_output), real_output)
	fake_loss = cross_entropy(tf.zeros_like(fake_output), fake_output)
	return real_loss + fake_loss

def generator_loss(fake_output):
	cross_entropy = tf.keras.losses.BinaryCrossentropy(from_logits=False)
	return cross_entropy(tf.ones_like(fake_output), fake_output)

# @tf.function
def custom_train_step(frames, generator, discriminator, generator_optimizer, discriminator_optimizer, batch_size, noise_size):
	noise = tf.random.normal([batch_size, noise_size])

	with tf.GradientTape() as gen_tape, tf.GradientTape() as disc_tape:
		generated_frames = generator(noise, training = True)

		real_output = discriminator(frames, training = True)
		fake_output = discriminator(generated_frames, training = True)

		gen_loss = generator_loss(fake_output)
		disc_loss = discriminator_loss(real_output, fake_output)

		gradient_g = gen_tape.gradient(gen_loss, generator.trainable_variables)
		gradient_d = disc_tape.gradient(disc_loss, discriminator.trainable_variables)

		generator_optimizer.apply_gradients(zip(gradient_g, generator.trainable_variables))
		discriminator_optimizer.apply_gradients(zip(gradient_d, discriminator.trainable_variables))

		return gen_loss, disc_loss

def train(dataset, generator, discriminator, generator_optimizer, discriminator_optimizer, checkpoint, checkpoint_prefix, batch_size, epochs, noise_size):
	history = {}
	history['generator_loss'] = []
	history['discriminator_loss'] = []
	for epoch in range(epochs):
		print(datetime.now().strftime(f'Epoch: {epoch}\t%Y%m%d_%H%M%S'))
		history['generator_loss'].append(0)
		history['discriminator_loss'].append(0)
		for i, batch in enumerate(dataset):
			print(f'Batch: {i}', end = '\r')
			gen_loss, disc_loss = custom_train_step(batch, generator, discriminator, generator_optimizer, discriminator_optimizer, batch_size, noise_size)
			# print(f'Generator loss: {gen_loss}\tDiscriminator loss: {disc_loss}')
			history['generator_loss'][-1] += gen_loss
			history['discriminator_loss'][-1] += disc_loss
		print(f'Batch: {i}')

		if (epoch + 1) % 10 == 0:
			checkpoint.save(file_prefix = checkpoint_prefix)
	print(f'Training complete after {epochs} epochs.')

	return history

def main():
	train_ds = None
	maximum = np.empty(0)
	for ds_file in os.listdir(sys.argv[1]):
		print(f'Loading file: {ds_file}')
		new_ds = Dataset.load(sys.argv[1] + '/' + ds_file)
		# remove empty angle values from imu and positions from odometry
		new_ds = new_ds.map(lambda x: x[21:24])
		# recalculate maximum
		if maximum.size > 0:
			new_maximum = np.absolute(np.array(list(new_ds))).max(axis = 0)
			maximum = np.array([maximum, new_maximum]).max(axis = 0)
			del new_maximum
		else:
			maximum = np.absolute(np.array(list(new_ds))).max(axis = 0)
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

	output_folder = datetime.now().strftime(f'odometry_anomaly_detector/imu_GAN/{len(list(train_ds))}_%Y%m%d_%H%M%S/')
	try:
		os.makedirs(output_folder)
	except:
		pass

	with open(output_folder + 'maximum.txt', 'w') as f:
		f.write(' '.join([str(m) for m in maximum]))

	train_ds.save(output_folder + 'train_ds')
	train_ds = train_ds.batch(BATCH_SIZE).prefetch(1)

	generator = make_generator_model(input_size = 3, output_size = 3)
	discriminator = make_discriminator_model(input_size = 3)
	generator_optimizer = tf.keras.optimizers.Adam(1e-4)
	discriminator_optimizer = tf.keras.optimizers.Adam(1e-4)

	# checkpoint_dir = data_dir[:-4] + '_checkpoints'
	checkpoint_prefix = os.path.join(output_folder, "ckpt")
	checkpoint = tf.train.Checkpoint(generator_optimizer=generator_optimizer,
								discriminator_optimizer=discriminator_optimizer,
								generator=generator,
								discriminator=discriminator)

	history = train(train_ds,
		generator,
		discriminator,
		generator_optimizer,
		discriminator_optimizer,
		checkpoint,
		checkpoint_prefix,
		batch_size = BATCH_SIZE,
		epochs = EPOCHS,
		noise_size = 3)

	plt.figure()
	plt.plot(history['generator_loss'], label = 'Generator loss')
	plt.plot(history['discriminator_loss'], label = 'Discriminator loss')
	plt.legend()
	plt.savefig(f'{output_folder}/plot.eps')
	plt.savefig(f'{output_folder}/plot.png')
	# plt.show()
	plt.close()

	for batch in train_ds:
		real_output = discriminator(batch)
		noise = tf.random.normal([BATCH_SIZE, 3])
		generated_frames = generator(noise, training = False)
		fake_output = discriminator(generated_frames, training = False)
		plt.clf()
		plt.plot(real_output, label = f'Real frames discriminator output, average {tf.reduce_mean(real_output)}')
		plt.plot(fake_output, label = f'Fake frames discriminator output, average {tf.reduce_mean(fake_output)}')
		plt.legend()
		plt.savefig(f'{output_folder}/test_batch.eps')
		plt.savefig(f'{output_folder}/test_batch.png')
		# plt.show()
		plt.close()
		break

if __name__ == '__main__':
	main()