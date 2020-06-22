import numpy as np
import scipy.signal as signal

def gaussian_filter(vec_array,window_size = 11,std = 1):
    gaussian_window = signal.gaussian(window_size,std)
    gaussian_window = gaussian_window/np.sum(gaussian_window)

    return signal.filtfilt(gaussian_window,[1],vec_array,axis = 0)


def discrete_gaussian_filter(label_arrays,empty_label = '',probability_threshold = 0.5):
    label_names,ids = np.unique(label_arrays,return_inverse=True)
    # print label_names
    this_shape = label_arrays.shape
    ids_arrays = ids.reshape(this_shape)

    occurance_arrays = []

    for id in range(len(label_names)):

        single_ids_array = np.transpose(np.array(id == ids_arrays).astype(int))
        occurance_array = (np.sum(single_ids_array,axis = 1)>0).astype(int)

        if label_names[id] == empty_label:
            occurance_array = (np.sum(single_ids_array, axis=1) < -1).astype(int)
            # print occurance_array

        occurance_arrays.append(occurance_array)
    probabilities = gaussian_filter(np.transpose(occurance_arrays))
    empty_label_samples = probability_threshold > np.max(gaussian_filter(np.transpose(occurance_arrays)),axis = 1)

    filtered_labels = label_names[np.argmax(gaussian_filter(np.transpose(occurance_arrays)),axis = 1)]
    filtered_labels[empty_label_samples] = empty_label
    return filtered_labels,probabilities



if __name__ == "__main__":

    stuff = np.array([['','','','','','a','a','a']*10,['','','b','b','b','','b','b']*10,['','','','c','c','c','c','']*10]).astype('S5')
    # print np.transpose(stuff)
    labels,probs = discrete_gaussian_filter(stuff)
    print labels