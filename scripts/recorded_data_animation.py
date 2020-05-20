import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import numpy as np
import os
import time
import keyboard


class SkeletonAnimation:
    def __init__(self, txt_file, num_sensors, num_paras, num_samples, n_sample, num_frames, interval):
        self.txt_file = txt_file
        self.num_sensors = num_sensors
        self.num_paras = num_paras
        self.num_samples = num_samples  # total number of samples
        self.n_sample = n_sample        # the index of sample to be plotted
        self.num_frames = num_frames
        self.interval = interval

        self.file_array = self.preprocess()
        # self.body_parts = ['head', 'neck', 'chest', 'spine', 'hip',                         # central part
        #                    'leftUpLeg', 'leftLeg', 'leftFoot',                              # left leg
        #                    'rightUpLeg', 'rightLeg', 'rightFoot',                           # right leg
        #                    'leftShoulder', 'leftUpperArm', 'leftLowerArm', 'leftHand',      # left arm
        #                    'rightShoulder', 'rightUpperArm', 'rightLowerArm', 'rightHand']  # right arm
        self.connections = [[0, 1], [1, 2], [2, 3], [3, 4],         # central part
                            [4, 5], [5, 6], [6, 7],                 # left leg
                            [4, 8], [8, 9], [9, 10],                # right leg
                            [2, 11], [11, 12], [12, 13], [13, 14],  # left arm
                            [2, 15], [15, 16], [16, 17], [17, 18]]  # right arm
        self.all_frame_linedata = self.get_all_frame_linedata()
        self.fig = plt.figure()
        self.ax = p3.Axes3D(self.fig)
        self.ax.set_xlim3d([-0.5, 0.5])
        self.ax.set_xlabel('X(m)')
        self.ax.set_ylim3d([-1, 0.8])
        self.ax.set_ylabel('Y(m)')
        self.ax.set_zlim3d([-0.2, 1.6])
        self.ax.set_zlabel('Z(m)')
        self.all_objects = self.create_all_objects()

        self.frame_idx_temp = [0]
        self.frame_idx = []
        self.c = 0
        self.start_time = time.time()
        self.ani = animation.FuncAnimation(self.fig, self.update_frame, self.num_frames,
                                           interval=self.interval, blit=True)

    def preprocess(self):
        """
        Process the txt file that stores raw suit data (from 'online_data_recording.py').
        """
        npy_file = '{}'.format(self.txt_file[0:-3]) + 'npy'

        if os.path.isfile(npy_file):
            return np.load(npy_file)
        else:
            file_array = np.zeros((self.num_samples, self.num_frames, self.num_sensors, self.num_paras))
            f = open(self.txt_file, 'r')
            f1 = f.readlines()
            n = 0
            for x in f1:
                one_sample = eval(x)
                for i in range(num_frames):
                    one_frame = one_sample[i]
                    frame_array = np.reshape(one_frame, (num_sensors, num_paras))
                    file_array[n, i, :, :] = frame_array
                n += 1
            f.close()
            np.save(npy_file, file_array)
            print('Transformed data from \'{}\' into shape {}'.format(self.txt_file, file_array.shape))
            # (num_samples, num_frames, num_sensors, num_paras) e.g., (50, 1000, 19, 3)
            return file_array

    def get_all_frame_linedata(self):
        """
        Prepare the data for plotting all skeleton connection lines in a record.
        """
        all_frame_linedata = []
        for i in range(self.file_array.shape[1]):
            # each frame

            xs = []
            ys = []
            zs = []
            LineData = []
            for j in range(self.file_array.shape[2]):
                # each sensor

                xs.append(self.file_array[n_sample, i, j, 0])
                ys.append(self.file_array[n_sample, i, j, 1])
                zs.append(self.file_array[n_sample, i, j, 2])

            for connection in self.connections:
                linedata = []
                xpoints = [xs[connection[0]],
                           xs[connection[1]]]
                ypoints = [ys[connection[0]],
                           ys[connection[1]]]
                zpoints = [zs[connection[0]],
                           zs[connection[1]]]
                linedata.append(xpoints)
                linedata.append(ypoints)
                linedata.append(zpoints)
                LineData.append(linedata)
            all_frame_linedata.append(LineData)
        return all_frame_linedata

    def create_all_objects(self):
        """
        Initialize all objects in the plot.
        """
        all_objects = []

        # create line objects
        # NOTE: Can't pass empty arrays into 3d version of plot()
        for n_line in range(self.num_sensors - 1):
            LineData = self.all_frame_linedata[0][n_line]
            line_obj = self.ax.plot(LineData[0], LineData[2], LineData[1], 'gray')[0]
            all_objects.append(line_obj)

        # create text object
        text_obj = self.ax.text(0, 0, 2.0, 'No.1 Record: 0.00 s', size=7, color='k')
        all_objects.append(text_obj)
        return all_objects

    def update_frame(self, i):
        """
        Update the objects to be plotted in each frame.
        """
        for n_line in range(len(self.all_objects) - 1):
            # NOTE: there is no .set_data() for 3 dim data...
            # (x, z, y) instead of (x, y, z)
            self.all_objects[n_line].set_data(self.all_frame_linedata[i][n_line][0], self.all_frame_linedata[i][n_line][2])
            self.all_objects[n_line].set_3d_properties(self.all_frame_linedata[i][n_line][1])
        self.all_objects[n_line + 1].set_text(u'Record {}: {:.2f} s'.format(n_sample + 1, i / 100))
        if keyboard.is_pressed('r'):  # if key 'r' is pressed
            if i > self.frame_idx_temp[-1]+10:
                print('Current frame: {}'.format(i))
                self.frame_idx_temp.append(i)
        if i == len(self.all_frame_linedata) - 1:
            print('Total time: {:.2f} s'.format(time.time() - self.start_time))
            self.frame_idx.append(self.frame_idx_temp)
            self.frame_idx_temp = [0]
            self.c += 1
        if self.c == 2:
            print('Please close the plot window.')
        return self.all_objects


def get_final_frame_idx(frame_idx):
    if len(frame_idx) != 2:
        print('Got {} frame_idx'.format(len(frame_idx)))
        return None
    temp1 = frame_idx[0]
    temp2 = frame_idx[1]
    return [int(sum(x) / 2) for x in zip(temp1, temp2)]


def save_frame_idx(txt_file, num_samples, n_sample, num_motions, frame_idx_list):
    npy_file = '{}'.format(txt_file[0:-4]) + '_frame_idx.npy'
    if not os.path.isfile(npy_file):
        final_frame_idx_array = np.zeros((num_samples, num_motions + 1))
        np.save(npy_file, final_frame_idx_array)
    final_frame_idx_array = np.load(npy_file)
    frame_idx = get_final_frame_idx(frame_idx_list)
    if frame_idx is not None and frame_idx != [0] * (num_motions + 1):
        final_frame_idx_array[n_sample] = frame_idx
        print('The separation points for record {} is {}'.format(n_sample, frame_idx))
        np.save(npy_file, final_frame_idx_array)
        print('Saved successfully!')
    else:
        print('Not saved!')


if __name__ == "__main__":

    txt_file = '50records_small_left.txt'
    num_sensors = 19
    num_paras = 3
    num_samples = 50  # total number of samples
    n_sample = 49     # the index of sample to be plotted
    num_frames = 1000
    interval = 10     # frame interval
    num_motions = 3

    sa = SkeletonAnimation(txt_file, num_sensors, num_paras, num_samples, n_sample, num_frames, interval)
    plt.show()

    save_frame_idx(txt_file, num_samples, n_sample, num_motions, sa.frame_idx)
