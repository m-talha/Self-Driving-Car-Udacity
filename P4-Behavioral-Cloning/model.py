import csv
from pathlib import Path
import cv2
import numpy as np

from keras import Sequential, Model
from keras.layers import Flatten, Dense, Lambda, Cropping2D, MaxPooling2D
from keras.layers.convolutional import Conv2D
from keras.optimizers import Adam

def read_csv(fpath):
    """Read csv file returning lines as a list."""    
    lines = []
    # Read in the csv file data
    try:
        with open(fpath) as csvfile:
            reader = csv.reader(csvfile)
            for line in reader:
                lines.append(line)
    except IOError as e:
        print("Couldn't read CSV data file: {}".format(e))
        print("Exiting...")
        exit()

    return lines

def load_images(inp_lines, img_fp, corr=0.2):
    """
    Load images and corresponding steering angles and returns as a list.

    Loads 3 images (center, left, right) for each line generating 
    measurements for left and right images using the @corr arg. Stores 
    each image and corresponding measurement in separate lists.

    Params:
    inp_lines (list) : Lines containing image paths and car control values.
    img_fp (str) : path to the folder containing track images
    corr (float) : correction to apply (additive). Default: 0.2

    Returns:
    images (list): Images of the track from 3 camera angles
    measurements (list): Corresponding steering angles
    """
    images = []
    measurements = []
    for line in inp_lines:
        # Include the left and right camera images
        for i in range(3):
            # Extract filename for each image as the last token
            source_path = line[i] # image path
            filename = source_path.split('\\')[-1] # image filename
            local_path = Path(img_fp).joinpath(filename) # create rel. path to filename
            image = cv2.cvtColor(cv2.imread(str(local_path)), cv2.COLOR_BGR2RGB)
            images.append(image)

        # Append augmented measurements for left and right images
        measurements.append(float(line[3])) 
        measurements.append(float(line[3]) + corr) # l_image: oversteer to right 
        measurements.append(float(line[3]) - corr) # r_image: understeer to left

    return images, measurements

def augment_images(imgs, measurements):
    """
    Augment data with vertically flipped images.

    Balances the data set to avoid bias towards one driving direction. Flipped 
    images are added with corresponding inverted measurements. 

    Params:
    imgs (list): images to be augmented
    measurements (list): corresponding measurements

    Returns:
    aug_images (list): image list with each image having an augmented duplicate
    aug_measurements (list): inverted measurements
    """
    aug_images = []
    aug_measurements = []
    for image, measurement in zip(imgs, measurements):
        aug_images.append(image)
        # Flip image around vertical axis
        aug_images.append(cv2.flip(image, 1))
        aug_measurements.append(measurement)
        # Invert measurement to match image
        aug_measurements.append(-1.0 * float(measurement))

    return aug_images, aug_measurements

def create_nvidiaNet(height, width, depth, tgt_dim):
    """NVidia's end-to-end deep learning model for self-driving cars"""
    model = Sequential()

    # Cropping layer - BEST EFFECT!!
    model.add(Cropping2D(cropping=((70, 20), (0, 0)), \
        input_shape=(height, width, depth)))
    # Normalisation layer
    model.add(Lambda(lambda x: (x / 255) - 0.5))

    # 5x Convolution layers
    model.add(Conv2D(24, (5, 5), strides=2, activation="relu"))
    model.add(Conv2D(36, (5, 5), strides=2, activation="relu"))
    model.add(Conv2D(48, (5, 5), strides=2, activation="relu"))
    model.add(Conv2D(64, (3, 3), activation="relu"))
    model.add(Conv2D(64, (3, 3), activation="relu"))

    # 3x fully connected layers
    model.add(Flatten())
    model.add(Dense(100))
    model.add(Dense(50))
    model.add(Dense(10))
    model.add(Dense(tgt_dim))

    return model

def create_leNet(height, width, depth, tgt_dim):
    """LeNet model for image classification"""
    model = Sequential()

    # Cropping layer - BEST EFFECT!!
    model.add(Cropping2D(cropping=((70, 20), (0, 0)), \
        input_shape=(height, width, depth)))
    # Normalisation layer
    model.add(Lambda(lambda x: (x / 255) - 0.5, \
        input_shape=(height, width, depth)))

    # 2x Convolution layers
    model.add(Conv2D(6, (5, 5), activation="relu"))
    model.add(MaxPooling2D(2))
    model.add(Conv2D(16, (5, 5), activation="relu"))
    model.add(MaxPooling2D(2))

    # 3x fully connected layers
    model.add(Flatten())
    model.add(Dense(120))
    model.add(Dense(84))
    model.add(Dense(tgt_dim))

    return model
    
def save_images(orig, aug, suffix=""):
    """
    Saves an example image and its augmented counterpart in the "./writeup_imgs" dir
    """
    fp = Path(__file__).parent.joinpath("writeup_imgs")

    try:
        cv2.imwrite(str(fp.joinpath(suffix+"_orig.jpg")), cv2.cvtColor(orig, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(fp.joinpath(suffix+"_aug.jpg")), cv2.cvtColor(aug, cv2.COLOR_RGB2BGR))
    except (IOError, OSError) as e:
        print("Unable to save example files: {}".format(e))

def main():
    """Entry point. Loads training data, trains a model and saves it before exiting"""
    # Names of files and folders to load input data from
    data_folder = "data"
    data_file = "driving_log.csv"
    images_folder = "IMG"

    # Path to the data folder
    data_fp = Path(__file__).parent.joinpath(data_folder, data_file)
    # Path to the image folder
    images_fp = Path(__file__).parent.joinpath(data_folder, images_folder)

    # Read file and load images
    lines = read_csv(data_fp)
    images, angles = load_images(lines, images_fp)
    # Augment the images
    aug_images, aug_measurements = augment_images(images, angles)

    # # Save example images- flip
    # save_images(aug_images[0], aug_images[1], "flip")
    # # Save example images- crop
    # t_img = aug_images[0][70:-20, :, :]
    # save_images(aug_images[0], t_img, "crop")

    # Create training data in format required by Keras
    X_train = np.array(aug_images)
    y_train = np.array(aug_measurements)
    img_height, img_width, img_depth = X_train.shape[1:]

    assert(len(X_train) > 0 and len(y_train) > 0)
    assert(len(X_train) == len(y_train))

    # Create the CNN model
    # model = create_leNet(img_height, img_width, img_depth, y_train.ndim)
    model = create_nvidiaNet(img_height, img_width, img_depth, y_train.ndim)
    # Compile and train model
    model.compile('adam', 'mse')
    model.fit(X_train, y_train, validation_split=0.2, shuffle=True, epochs=5)

    # Save model
    model.save('model.h5')

if __name__ == "__main__":
    main()