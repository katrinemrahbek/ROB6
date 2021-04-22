import os
import glob
import trimesh
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from matplotlib import pyplot as plt
import random
import open3d as o3d

tf.random.set_seed(1234)
Data_dir = "D:\SyncFolder\P6\ProjectFiles\PipeAI\Dataset"


def augment(points, label):
    # jitter points
    points += tf.random.uniform(points.shape, -0.005, 0.005, dtype=tf.float64)
    # shuffle points
    points = tf.random.shuffle(points)
    return points, label


def conv_bn(x, filters):
    x = layers.Conv1D(filters, kernel_size=1, padding="valid")(x)
    x = layers.BatchNormalization(momentum=0.0)(x)
    return layers.Activation("relu")(x)

def dense_bn(x, filters):
    x = layers.Dense(filters)(x)
    x = layers.BatchNormalization(momentum=0.0)(x)
    return layers.Activation("relu")(x)

class OrthogonalRegularizer(keras.regularizers.Regularizer):
    def __init__(self, num_features, l2reg=0.001):
        self.num_features = num_features
        self.l2reg = l2reg
        self.eye = tf.eye(num_features)

    def __call__(self, x):
        x = tf.reshape(x, (-1, self.num_features, self.num_features))
        xxt = tf.tensordot(x, x, axes=(2, 2))
        xxt = tf.reshape(xxt, (-1, self.num_features, self.num_features))
        return tf.reduce_sum(self.l2reg * tf.square(xxt - self.eye))

    def get_config(self):
        return {"num_features": self.num_features, "l2reg": self.l2reg}

    @classmethod
    def from_config(cls, config):
        return cls(**config)

def tnet(inputs, num_features):
    # Initalise bias as the indentity matrix
    bias = keras.initializers.Constant(np.eye(num_features).flatten())
    reg = OrthogonalRegularizer(num_features)

    x = conv_bn(inputs, 32)
    x = conv_bn(x, 64)
    x = conv_bn(x, 512)
    x = layers.GlobalMaxPooling1D()(x)
    x = dense_bn(x, 256)
    x = dense_bn(x, 128)
    x = layers.Dense(
        num_features * num_features,
        kernel_initializer="zeros",
        bias_initializer=bias,
        activity_regularizer=reg,
    )(x)
    feat_T = layers.Reshape((num_features, num_features))(x)
    # Apply affine transformation to input features
    return layers.Dot(axes=(2, 1))([inputs, feat_T])

def parse_dataset(num_points=2048):
    folders = os.listdir(Data_dir)
    train_points = []
    train_labels = []
    test_points = []
    test_labels = []
    class_map = {}
    i = 0;

    for folder in folders:
        print("processing class: {}".format(folder))
        # store folder name with ID so we can retrieve later


        is_found = False
        gotten = class_map.values()
        for j, key in enumerate(gotten):
            if key == folder[0:7]:
                i = j
                is_found = True

        if is_found == False:
            i = len(class_map)
            class_map.update({i: folder[0:7]})

        train_files = []
        test_files = []
        # gather all files
        for subfolders in os.listdir(Data_dir + "/" + folder):
            for files in os.listdir(Data_dir + "/" + folder + "/" + subfolders):
                train_files.append(Data_dir + "/" + folder + "/" + subfolders + "/" + files)

        for train_index in range(0, int(len(train_files) / 10)):
            the_number = random.randint(0, len(train_files)-1)
            test_files.append(train_files.pop(the_number))

        for f in train_files:
            cloud = o3d.io.read_point_cloud(f)
            if len(cloud.points) == 0:
                continue
            random_numbers = random.sample(range(0, len(cloud.points)), num_points)
            train_points.append(np.asarray(cloud.points)[random_numbers])
            train_labels.append(i)

        for f in test_files:
            cloud = o3d.io.read_point_cloud(f)
            if len(cloud.points) == 0:
                continue
            random_numbers = random.sample(range(0, len(cloud.points)-1), num_points)
            test_points.append(np.asarray(cloud.points)[random_numbers])
            test_labels.append(i)

    return (
        np.array(train_points),
        np.array(test_points),
        np.array(train_labels),
        np.array(test_labels),
        class_map,
    )

NUM_POINTS = 2048
BATCH_SIZE = 32

train_points, test_points, train_labels, test_labels, CLASS_MAP = parse_dataset(
    NUM_POINTS
)
NUM_CLASSES = len(CLASS_MAP)
train_dataset = tf.data.Dataset.from_tensor_slices((train_points, train_labels))
test_dataset = tf.data.Dataset.from_tensor_slices((test_points, test_labels))

train_dataset = train_dataset.shuffle(len(train_points)).map(augment).batch(BATCH_SIZE)
test_dataset = test_dataset.shuffle(len(test_points)).batch(BATCH_SIZE)

inputs = keras.Input(shape=(NUM_POINTS, 3))


x = tnet(inputs, 3)
x = conv_bn(x, 32)
x = conv_bn(x, 32)
x = tnet(x, 32)
x = conv_bn(x, 32)
x = conv_bn(x, 64)
x = conv_bn(x, 512)
x = layers.GlobalMaxPooling1D()(x)
x = dense_bn(x, 256)
x = layers.Dropout(0.3)(x)
x = dense_bn(x, 128)
x = layers.Dropout(0.3)(x)

outputs = layers.Dense(NUM_CLASSES, activation="softmax")(x)

model = keras.Model(inputs=inputs, outputs=outputs, name="pointnet")
model.summary()

model.compile(
    loss="sparse_categorical_crossentropy",
    optimizer=keras.optimizers.Adam(learning_rate=0.001),
    metrics=["sparse_categorical_accuracy"],
)

model.fit(train_dataset, epochs=20, validation_data=test_dataset)

data = test_dataset.take(1)

points, labels = list(data)[0]
points = points[:8, ...]
labels = labels[:8, ...]

model.save('D:/SyncFolder/P6/ProjectFiles/PipeAI')

# run test data through model
preds = model.predict(points)
preds = tf.math.argmax(preds, -1)

points = points.numpy()

# plot points with predicted class and label
fig = plt.figure(figsize=(15, 10))
for i in range(8):
    ax = fig.add_subplot(2, 4, i + 1, projection="3d")
    ax.scatter(points[i, :, 0], points[i, :, 1], points[i, :, 2])
    ax.set_title(
        "pred: {:}, label: {:}".format(
            CLASS_MAP[preds[i].numpy()], CLASS_MAP[labels.numpy()[i]]
        )
    )
    ax.set_axis_off()
plt.show()




