import numpy as np
import pickle
import collections
import matplotlib.pyplot as plt
import random
import os
import glob
import copy
from sklearn.metrics import confusion_matrix
import seaborn as sn
import pandas as pd


class PreProcessing:
    """
    Process a txt file where each line is a complete record,
    in order to construct suitable datasets for the motion classification model.
    Options:
    1. Remove abnormal records:
            clean(self, txt_file, num_records, record_idx=[])
    2. Split the records into train/val/test sets:
            split(self, txt_file, num_records, proportions)
    3. Plot the histograms of all records or records in a subset:
            plot_hist(self, frame_idx_file, num_frames_per_clip, titles=[])
    4. Segment each record into motions, then downsample each motion into clips:
            segment(self)
    """
    def __init__(self, txt_file, num_records, num_sensors, num_paras, num_frames, num_frames_per_clip, step, motion_clips_dict):
        self.txt_file = txt_file
        self.num_records = num_records
        self.num_sensors = num_sensors
        self.num_paras = num_paras
        self.num_frames = num_frames
        self.num_frames_per_clip = num_frames_per_clip
        self.step = step
        self.motion_clips_dict = motion_clips_dict

        self.data = self.transform_txt_file()
        # self.segment()

    def transform_txt_file(self):
        """
        Transform lines in the txt file to an array.
        """
        file_arr = np.zeros((self.num_records, self.num_frames, self.num_sensors, self.num_paras))
        f = open(self.txt_file, 'r')
        records = f.readlines()
        n = 0
        for record in records:
            one_record = eval(record)
            for i in range(self.num_frames):
                one_frame = one_record[i]
                frame_arr = np.reshape(one_frame, (self.num_sensors, self.num_paras))
                file_arr[n, i, :, :] = frame_arr
            n += 1
        f.close()
        arr1 = np.swapaxes(file_arr, 1, 3)
        arr2 = np.swapaxes(arr1, 2, 3)
        arr3 = arr2[:, :, :, :, np.newaxis]
        # npy_file = '{}'.format(file_name[0:-3]) + 'npy'
        # np.save(npy_file, arr3)
        print('Transformed lines in \'{}\' into an array with shape {}'.format(self.txt_file, arr3.shape))
        # (num_records, num_paras, num_frames, num_sensors, 1) e.g., (50, 3, 1000, 19, 1)
        return arr3

    def downsample(self, motion, j):
        """
        Downsample the motion in the dimension of 'num_frames'.
        """
        n_frames = motion.shape[1]
        n_clips = int((n_frames - self.num_frames_per_clip) // self.step + 1)
        sampled_motion = np.zeros((n_clips, self.num_paras, self.num_frames_per_clip, self.num_sensors, 1))
        for n in range(n_clips):
            sampled_motion[n] = motion[:, (n * self.step):(n * self.step + self.num_frames_per_clip), :, :]
        print('{}: {}'.format(list(self.motion_clips_dict.keys())[j], sampled_motion.shape))
        return sampled_motion

    def segment(self):
        """
        Segment each record into motions, then downsample each motion and store the generated clips in a dictionary.
        """
        frame_idx_file = '{}'.format(self.txt_file[0:-4]) + '_frame_idx.npy'
        frame_idx_arr = np.load(frame_idx_file)
        clip_count_file = '{}'.format(self.txt_file[0:-4]) + '_clip_count.pickle'

        for i in range(self.data.shape[0]):
            # each record
            print('Record {}: '.format(i))
            record = self.data[i]
            frame_idx = frame_idx_arr[i]
            for j in range(len(frame_idx)-1):
                # each motion
                motion = record[:, int(frame_idx[j]):int(frame_idx[j+1]), :, :]
                sampled_motion = self.downsample(motion, j)
                # save the number of clips for each motion in test set
                if self.txt_file[-8:-4] == 'test':
                    if not os.path.isfile(clip_count_file):
                        clip_count_dict = motion_clips_dict.copy()
                        for k in clip_count_dict.keys():
                            clip_count_dict[k] = []
                        with open(clip_count_file, 'wb') as f:
                            pickle.dump(clip_count_dict, f)
                    with open(clip_count_file, 'rb') as f:
                        clip_count_dict = pickle.load(f)
                    clip_count_dict[list(clip_count_dict.keys())[j]].append(sampled_motion.shape[0])
                    with open(clip_count_file, 'wb') as f:
                        pickle.dump(clip_count_dict, f)
                self.motion_clips_dict[list(self.motion_clips_dict.keys())[j]].append(sampled_motion)

        # print out summary
        print('\n\nShape of each clip: \n\t{}\nTotal clips for each motion class in \'{}\': '
              .format((self.num_paras, self.num_frames_per_clip, self.num_sensors, 1), self.txt_file))
        for k, v in self.motion_clips_dict.items():
            v_new = np.concatenate(v, axis=0)
            self.motion_clips_dict[k] = v_new
            print('\t{}: {}'.format(k, self.motion_clips_dict[k].shape[0]))

        # save clips dictionary
        motion_clips_file = '{}'.format(self.txt_file[0:-4]) + '_motion_clips.pickle'
        with open(motion_clips_file, 'wb') as f:
            pickle.dump(self.motion_clips_dict, f)

    def write_txt_file(self, txt_file, new_txt_file, record_idx):
        f = open(txt_file, 'r')
        records = f.readlines()
        f_new = open(new_txt_file, 'w')
        n = 0
        for record in records:
            if n in record_idx:
                f_new.write(record)
            n += 1
        f_new.close()
        f.close()
        print('Saved {}: {} records.'.format(new_txt_file, len(record_idx)))

    def write_npy_file(self, frame_idx_file, new_frame_idx_file, record_idx):
        frame_idx = np.load(frame_idx_file)
        new_frame_idx = frame_idx[record_idx]
        np.save(new_frame_idx_file, new_frame_idx)
        print('Saved {}: {} records.'.format(new_frame_idx_file, len(record_idx)))

    def clean(self, txt_file, num_records, record_idx=[]):
        """
        Remove bad records which are found by 'recorded_data_animation.py' and create new files.
        """
        frame_idx_file = '{}'.format(txt_file[0:-4]) + '_frame_idx.npy'
        if len(record_idx) == 0:
            return txt_file, frame_idx_file, num_records
        else:
            new_record_idx = list(set(range(num_records)) - set(record_idx))
            # modify txt file
            new_txt_file = '{}'.format(len(new_record_idx)) + '_{}'.format(txt_file)
            self.write_txt_file(txt_file, new_txt_file, new_record_idx)
            # modify frame_idx npy file
            new_frame_idx_file = '{}'.format(len(new_record_idx)) + '_{}'.format(frame_idx_file)
            self.write_npy_file(frame_idx_file, new_frame_idx_file, new_record_idx)
        return new_txt_file, new_frame_idx_file, len(new_record_idx)

    def split(self, txt_file, num_records, proportions):
        """
        Split the records into train/val/test sets and save each set separately.
        """
        assert proportions[0] != 0 and proportions[1] != 0, 'Can\'t handle proportions with 0: {}'.format(proportions)

        frame_idx_file = '{}'.format(txt_file[0:-4]) + '_frame_idx.npy'
        set_split_file = txt_file[0:txt_file.index('_')] + '_set_split.pickle'

        if not os.path.isfile(set_split_file):
            set_split = {'i_train': [], 'i_val': [], 'i_test': []}
            n_val, n_test = int(num_records * proportions[0]), int(num_records * proportions[1])
            n_train = num_records - n_val - n_test

            i_val = random.sample(range(num_records), k=n_val)
            i_test = random.sample(list(set(range(num_records)) - set(i_val)), k=n_test)
            i_train = list(set(range(num_records)) - set(i_val) - set(i_test))
            set_split['i_train'], set_split['i_val'], set_split['i_test'] = i_train, i_val, i_test

            with open(set_split_file, 'wb') as f:
                pickle.dump(set_split, f)
            print('Saved {}: {}'.format(set_split_file, set_split))
        with open(set_split_file, 'rb') as f:
            set_split = pickle.load(f)
        i_train, i_val, i_test = set_split['i_train'], set_split['i_val'], set_split['i_test']

        train_txt = '{}'.format(txt_file[0:-4]) + '_train.txt'
        val_txt = '{}'.format(txt_file[0:-4]) + '_val.txt'
        test_txt = '{}'.format(txt_file[0:-4]) + '_test.txt'
        train_frame_idx_npy = '{}'.format(train_txt[0:-4]) + '_frame_idx.npy'
        val_frame_idx_npy = '{}'.format(val_txt[0:-4]) + '_frame_idx.npy'
        test_frame_idx_npy = '{}'.format(test_txt[0:-4]) + '_frame_idx.npy'
        if not os.path.isfile(train_txt):
            self.write_txt_file(txt_file, train_txt, i_train)
            self.write_txt_file(txt_file, val_txt, i_val)
            self.write_txt_file(txt_file, test_txt, i_test)
            self.write_npy_file(frame_idx_file, train_frame_idx_npy, i_train)
            self.write_npy_file(frame_idx_file, val_frame_idx_npy, i_val)
            self.write_npy_file(frame_idx_file, test_frame_idx_npy, i_test)
        return train_txt, val_txt, test_txt, i_train, i_val, i_test

    def plot_hist(self, frame_idx_file, num_frames_per_clip,
                  titles=['\'pick up a box\'', '\'put the box\'', '\'finish\'']):
        """
        Plot the histograms of all records or each set (train/val/test).
        """
        frame_idx_arr = np.load(frame_idx_file)
        motions_lens = []
        for i in range(frame_idx_arr.shape[1]-1):
            motion_lens = frame_idx_arr[:, i + 1] - frame_idx_arr[:, i]
            for j in range(frame_idx_arr.shape[0]):
                if motion_lens[j] < num_frames_per_clip:
                    print('Record {} in motion {} has only {} frames'.format(j, i, motion_lens[j]))
            motions_lens.append(motion_lens)
        fig, axes = plt.subplots(1, 3)
        axes = axes.ravel()
        for idx, ax in enumerate(axes):
            ax.hist(motions_lens[idx])
            ax.set_title(titles[idx])
            ax.set_xlabel('frames')
            ax.set_ylabel('records')
        plt.tight_layout()
        plt.show()


def simple_packing_dicts(dict_opts):
    """
    Initialize a dictionary to store motion clips in the simple packing scenario.
    dict_opts specifies box size and box final position, e.g., ['small', 'left']
    """
    assert dict_opts[0] in ['small', 'big'] and dict_opts[1] in ['left', 'right'],\
        'Can\'t handle dict_opts: {}'.format(dict_opts)

    motion_clips_dict = collections.OrderedDict([('pick up the {} box'.format(dict_opts[0]), []),
                                                 ('put the box on {}'.format(dict_opts[1]), []),
                                                 ('finish', [])])

    data_dict = collections.OrderedDict([('pick up the small box', []),
                                         ('pick up the big box', []),
                                         ('put the box on left', []),
                                         ('put the box on right', []),
                                         ('finish', [])])
    return motion_clips_dict, data_dict


def finalize_subsets(subset, data_dict, dict_files):
    """
    Gather all motion clips from multiple files belonging to the same subset and create corresponding labels.
    """
    _data, data_file = [], '{}_data.npy'.format(subset)
    _label, label_file = [], '{}_label.npy'.format(subset)
    if subset == 'test':
        clip_count_dict = copy.deepcopy(data_dict)

    for dict_file in dict_files:
        with open(dict_file, 'rb') as f:
            dict = pickle.load(f)
        for k, v in dict.items():
            data_dict[k].append(v)

    c = 0
    print('Total clips for each motion class: ')
    for k, v in data_dict.items():
        v_new = np.concatenate(v, axis=0)
        data_dict[k] = v_new
        _data.append(v_new)
        _label.append([c]*v_new.shape[0])
        c += 1
        print('\t{}: {}'.format(k, v_new.shape[0]))

    data_arr = np.concatenate(_data, axis=0)
    np.save(data_file, data_arr)
    print('Saved {}'.format(data_file))

    label_arr = np.concatenate(_label, axis=0)
    np.save(label_file, label_arr)
    print('Saved {}'.format(label_file))

    data_dict_file = '{}_motion_clips_dict.pickle'.format(subset)
    with open(data_dict_file, 'wb') as f:
        pickle.dump(data_dict, f)
    print('Saved {}'.format(data_dict_file))

    if subset == 'test':
        for dict_file in dict_files:
            with open(dict_file[0:-20] + '_clip_count.pickle', 'rb') as f:
                dict = pickle.load(f)
            for k, v in dict.items():
                clip_count_dict[k].append(v)

        print('Clip counts of each motion for each motion class: ')
        clip_count = []
        for k, v in clip_count_dict.items():
            v_new = [item for sublist in v for item in sublist]
            clip_count_dict[k] = v_new
            clip_count.append(v_new)
            print('\t{}: {}'.format(k, v_new))

        clip_count_list = [item for sublist in clip_count for item in sublist]
        clip_count_file = '{}_clip_count_list.npy'.format(subset)
        np.save(clip_count_file, clip_count_list)
        print('Saved {}'.format(clip_count_file))


class PostProcessing:
    """
    Process results from the motion classification model.
    Options:
    1. Compute top1 acc:
            compute_top1_acc(self)
    2. Plot the confusion matrix:
            plot_confusion_matrix(self)
    3. Track clips results:
            track_clips(self)
    """
    def __init__(self, test_dict_file, test_label_file, top1_acc_file, pred_file, clip_count_file):
        self.test_dict = self.load_dict(test_dict_file)
        self.test_labels = np.load(test_label_file)
        self.top1_accs = np.load(top1_acc_file)
        self.preds = np.load(pred_file)
        self.clip_counts = np.load(clip_count_file)

        self.class_splits, self.class_names = [0], []
        self.load_class_info()

    def load_dict(self, dict_file):
        """
        Load a dictionary from a pickle file.
        """
        with open(dict_file, 'rb') as f:
            dict = pickle.load(f)
        print('Load ' + dict_file[0:-7] + ': ')
        for k, v in dict.items():
            print('\t{}: {}'.format(k, v.shape[0]))
        return dict

    def load_class_info(self):
        """
        Load the class splits and class names in the test set.
        """
        c = 0
        for k, v in self.test_dict.items():
            c += v.shape[0]
            self.class_splits.append(int(c))
            self.class_names.append(k)

    def compute_top1_acc(self):
        """
        Compute the top1 acc for each motion class in the test set.
        """
        i = 0
        print('Top1 accuracy for each motion class: ')
        for k, v in self.test_dict.items():
            current_accs = self.top1_accs[self.class_splits[i]:self.class_splits[i + 1]]
            n_correct = np.count_nonzero(current_accs == 100.0)
            top1_acc_class = n_correct / v.shape[0]
            print('\t{}:\t{:0.2f}'.format(k, top1_acc_class))
            i += 1

    def track_clips(self):
        """
        Print clip results row by row, each row corresponds to a motion record.
        """
        results = [0]*self.class_splits[-1]
        for i in range(self.class_splits[-1]):
            if self.top1_accs[i] == 100.0:
                results[i] = 1

        i = 0
        c = 0
        j = 1
        print('\nClips in each motion record (each row):')
        for k, v in self.test_dict.items():
            print(k)
            while i < self.class_splits[j]:
                print('\t{}'.format(results[i:i + self.clip_counts[c]]))
                i += self.clip_counts[c]
                c += 1
            j += 1

    def plot_confusion_matrix(self):
        """
        Plot the confusion matrix for the motion classification.
        """
        cm = confusion_matrix(self.test_labels.tolist(), self.preds)
        df_cm = pd.DataFrame(cm, index=self.class_names, columns=self.class_names)
        sn.set(font_scale=0.5)
        sn.heatmap(df_cm, cmap="YlGnBu", center=40, annot=True, fmt="d", annot_kws={"size": 12},
                   linewidths=0.5, cbar=False, square=True)
        plt.yticks(rotation=0)
        plt.show()


if __name__ == '__main__':

    random.seed(0)

    ###########################################################
    # A setup example for data cleaning and data set splitting.
    ###########################################################
    txt_file    = '50records_big_right.txt'                # a txt file that contains records before data set splitting
    num_records = 50                                       # total number of records in the txt file
    record_idx  = []                                       # a list of indexes of records to be removed
    proportions = [0.2, 0.1]                               # proportions of val set and test set
    num_frames_per_clip = 100                              # number of frames in each clip

    #######################################################
    # A setup example for data segmenting and downsampling.
    #######################################################
    txt_file    = '50records_big_right_test.txt'           # a txt file that contains records after data set splitting
    num_records = 5                                        # depends on proportions, e.g., train:val:test = 0.7:0.2:0.1
    num_sensors = 19                                       # number of sensors on the SmartSuit
    num_paras   = 3                                        # x, y, z
    num_frames  = 1000                                     # number of frames in each record
    step        = 10                                       # number of frames to move the clip window within motion data
    dict_opts   = ['big', 'right']                         # specifies box size and box final position
    motion_clips_dict, data_dict = simple_packing_dicts(dict_opts)
    # pre = PreProcessing(txt_file, num_records, num_sensors, num_paras, num_frames, num_frames_per_clip,
    #                     step, motion_clips_dict)
    # pre.clean(txt_file, num_records, record_idx)
    # pre.split(txt_file, num_records, proportions)
    # pre.segment()

    #######################################################
    # A setup example for final step of subset preparation.
    #######################################################
    subset      = 'test'                                   # specifies the name of that subset
    if subset == 'test':                                   # fix the order of clips to track misclassified ones
        dict_files = ['50records_small_left_test_motion_clips.pickle',
                      '50records_small_right_test_motion_clips.pickle',
                      '50records_big_left_test_motion_clips.pickle',
                      '50records_big_right_test_motion_clips.pickle']
    else:
        dict_files = []
        for file in glob.glob('*{}_motion_clips.pickle'.format(subset)):
            dict_files.append(file)
    # finalize_subsets(subset, data_dict, dict_files)

    #######################################
    # A setup example for results analysis.
    #######################################
    test_dict_file  = 'test_motion_clips_dict.pickle'
    test_label_file = 'test_label.npy'
    top1_acc_file   = 'top1_acc_clip.npy'
    pred_file       = 'pred_clip.npy'
    clip_count_file = 'test_clip_count_list.npy'
    post = PostProcessing(test_dict_file, test_label_file, top1_acc_file, pred_file, clip_count_file)
    # post.compute_top1_acc()
    post.track_clips()
    # post.plot_confusion_matrix()
