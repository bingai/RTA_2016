#!/usr/bin/env python
import os
import os.path
import rospy
import rospkg
import cv2
import random
import numpy
import scipy
import subprocess
import itertools
import glob
from cv_bridge import CvBridge, CvBridgeError

import time

import sys
if os.environ.has_key('CAFFE_ROOT'):
    caffe_root = os.environ['CAFFE_ROOT']
else:
    caffe_root = '/usr/local/caffe'
sys.path.append(os.path.join(caffe_root, 'python'))
import caffe

rospack = rospkg.RosPack()
pkg_root = rospack.get_path("object_cls_node")

class ObjectClassifier(object):
    def __init__(self):
        self._name = ''
        self._classes = ('NULL')
        self._cvbridge = CvBridge()
        self._padding_rate = 1.2
        self._patch_size = 224
        self._test_batch_size = 3
        # gpu_id = 0
        # caffe.set_mode_gpu()
        # caffe.set_device(gpu_id);

        self._image_count = 0
        self._image_cache_dir = os.path.join(pkg_root, 'cache')
        if not os.path.exists(self._image_cache_dir):
            os.makedirs(self._image_cache_dir)
        else:
            imfiles = glob.glob(os.path.join(self._image_cache_dir, '*.jpg'))
            self._image_count = len(imfiles)

    def run_classifier(self, data):
        raise NotImplementedError


    def dummy_classifier(self, image, bboxes):
        scores = []
        predictions = []
        for bb in bboxes:
            s = random.random()
            scores.append([random.random() for _ in self._classes])
            predictions.append(scores.index(scores))

        return numpy.asarray(scores, dtype='float32'), numpy.asarray(predictions, dtype='int32')

    def get_patch(self, np_image, bbox):
        height, width = np_image.shape[0:2]
        if bbox[0] >= width or bbox[2] <= 0 or bbox[1] >= height or bbox[3] <= 0:
            return numpy.zeros([self._patch_size, self._patch_size, 3], dtype='uint8')

        bbox_sz = bbox[2:4]-bbox[0:2]+1
        center = (bbox[0:2]+bbox[2:4])/2.0
        crop_sz = int(max(bbox_sz) * self._padding_rate)

        xs = numpy.asarray(center[0]-crop_sz/2.0+numpy.array(range(crop_sz)), dtype='int32')
        ys = numpy.asarray(center[1]-crop_sz/2.0+numpy.array(range(crop_sz)), dtype='int32')
        xs[xs<0] = 0
        ys[ys<0] = 0
        xs[xs>=width] = width-1
        ys[ys>=height] = height-1

        xmat = numpy.tile(xs, (crop_sz, 1)).T
        ymat = numpy.tile(ys, (crop_sz, 1))

        return scipy.misc.imresize(np_image[ymat, xmat], [self._patch_size, self._patch_size])


    def cnn_classifier(self, image, bboxes):
        try:
            cv_image = self._cvbridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print e

        np_image = numpy.asarray(cv_image)[:,:,::-1]
        # extract patches
        print "Extracting patches...",
        start_time = time.time()
        n_bbox = len(bboxes)

        # cache images
        disp_image = numpy.zeros_like(cv_image)
        numpy.copyto(disp_image, cv_image)
        for i in range(n_bbox):
            bbox = numpy.asarray(bboxes[i])
            cv2.rectangle(disp_image, tuple(bbox[0:2]), tuple(bbox[2:]), (0,0,255), 2)
            cv2.putText(disp_image, "bbox%d"%(i), (bbox[0], bbox[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 1)

        image_filename = os.path.join(self._image_cache_dir, 'images%06d_ori.jpg'%(self._image_count))
        cv2.imwrite(image_filename, cv_image)
        image_filename = os.path.join(self._image_cache_dir, 'images%06d_disp.jpg'%(self._image_count))
        cv2.imwrite(image_filename, disp_image)
        bbox_filename = os.path.join(self._image_cache_dir, 'images%06d_bbox.txt'%(self._image_count))
        with open(bbox_filename, 'w') as f:
            for i in range(n_bbox):
                f.write('%d %d %d %d\n' % (bbox[0], bbox[1], bbox[2], bbox[3]))
        self._image_count += 1


        # padding patches to match batch size
        n_batch = (n_bbox-1)//self._test_batch_size+1
        n_patch = n_batch * self._test_batch_size
        print 'n_patch: ', n_patch, 'batch_size: ', self._test_batch_size

        patches = numpy.zeros([n_patch, self._patch_size, self._patch_size, 3], dtype='uint8')
        for i in range(n_bbox):
            bbox = numpy.asarray(bboxes[i])
            patches[i] = self.get_patch(np_image, bbox)
        print "%.03fs" % (time.time() - start_time)

        scores = numpy.zeros((n_patch, len(self._classes)), dtype='float32')
        predictions = numpy.zeros((n_patch, 1), dtype='uint8')
        print "Batch processing...",
        start_time = time.time()
        for bi in range(n_batch):
            print "Processing patch %d" % (i)
            batch_idx = range(bi*self._test_batch_size, (bi+1)*self._test_batch_size)
            prob = self.run_classifier(patches[batch_idx].transpose((0,3,2,1)))

            scores[batch_idx] = prob
            predictions[batch_idx,0] = prob.argmax(axis=1)
        scores = scores[:n_bbox]
        predictions = predictions[:n_bbox]
        print "%.03fs" % (time.time() - start_time)

        return scores, predictions

    def classify_bbox_lists(self, images, bbox_lists):
        score_map = numpy.zeros((len(bbox_lists[0]), len(self._classes)), dtype='float32')

        for i in range(len(images)):
            score_map_new, pred = self.cnn_classifier(images[i], bbox_lists[i])
            score_map = numpy.maximum(score_map, score_map_new)

        scores = []
        predictions = []
        for i in range(len(bbox_lists[0])):
            scores.append(score_map[i].max())
            predictions.append(score_map[i].argmax())

        cls_counter = dict([(c, 0) for c in self._classes])
        prediction_strs = []
        for p in predictions:
            cls = self._classes[p]
            prediction_strs.append(cls+'%d'%cls_counter[cls])
            cls_counter[cls] += 1

        print scores
        print prediction_strs

        return scores, prediction_strs


class DishClassifier(ObjectClassifier):

    def __init__(self):
        super(DishClassifier, self).__init__()

        self._name = 'dish'
        self._classes = ('NULL', # always index 0
                         'bowl', 'cup', 'mug', 'plate')

        # self._proto_file = os.path.join(pkg_root, 'model', 'vgg_16', 'deploy.prototxt')
        # self._model_file = os.path.join(pkg_root, 'model', 'vgg_16', 'vgg_16_finetune_dish_iter_1000.caffemodel')
        self._proto_file = os.path.join(pkg_root, 'model', 'vgg_cnn_m_1024', 'deploy_cnn.prototxt')
        self._model_file = os.path.join(pkg_root, 'model', 'vgg_cnn_m_1024', 'vgg_cnn_m_1024_dish_iter_1000.caffemodel')
        self._net = caffe.Net(self._proto_file, self._model_file, caffe.TEST)
        self._net.blobs['data'].reshape(self._test_batch_size, 3, self._patch_size, self._patch_size)


    def run_classifier(self, data):
        self._net.blobs['data'].data[:] = data
        out = self._net.forward()
        prob = out['cls_prob'].astype('float32')
        return prob


class DishColorClassifier(ObjectClassifier):

    def __init__(self):
        super(DishColorClassifier, self).__init__()

        self._name = 'dishcolor'
        # self._classes = ('NULL', 'blue', 'red', 'yellow', 'pink', 'orange', 'green', 'brown', 'white')
        self._classes = ('NULL', 'blue', 'red', 'yellow', 'green', 'white', 'black', 'orange')

        self._cvbridge = CvBridge()
        self._padding_rate = 1.2
        self._patch_size = 224
        self._test_batch_size = 3

        # gpu_id = 0
        # caffe.set_mode_gpu()
        # caffe.set_device(gpu_id);

        self._proto_file = os.path.join(pkg_root, 'model', 'dishcolor', 'deploy_cnn.prototxt')
        # self._model_file = os.path.join(pkg_root, 'model', 'dishcolor', 'vgg_cnn_m_1024_dishcolor_iter_1000.caffemodel')
        self._model_file = os.path.join(pkg_root, 'model', 'dishcolor', 'vgg_cnn_m_1024_dishcolor_dual_iter_2000.caffemodel')
        self._net = caffe.Net(self._proto_file, self._model_file, caffe.TEST)
        self._net.blobs['data'].reshape(self._test_batch_size, 3, self._patch_size, self._patch_size)


    def run_classifier(self, data):
        self._net.blobs['data'].data[:] = data
        out = self._net.forward()
        prob = out['color_cls_prob'].astype('float32')
        return prob



class DishDualClassifier(ObjectClassifier):

    def __init__(self):
        super(DishDualClassifier, self).__init__()

        self._name = 'dishdual'
        self._obj_classes = ('NULL', # always index 0
                         'bowl', 'cup', 'mug', 'plate')
        # self._classes = ('other', 'blue', 'red', 'yellow', 'pink', 'orange', 'green', 'brown', 'white')
        self._color_classes = ('NULL', 'blue', 'red', 'yellow', 'green', 'white', 'black', 'orange')

        self._classes = ['.'.join(x) for x in itertools.product(self._obj_classes, self._color_classes)]

        self._cvbridge = CvBridge()
        self._padding_rate = 1.2
        self._patch_size = 224
        self._test_batch_size = 3

        # gpu_id = 0
        # caffe.set_mode_gpu()
        # caffe.set_device(gpu_id);

        self._proto_file = os.path.join(pkg_root, 'model', 'dishcolor', 'deploy_cnn.prototxt')
        # self._model_file = os.path.join(pkg_root, 'model', 'dishcolor', 'vgg_cnn_m_1024_dishcolor_iter_1000.caffemodel')
        self._model_file = os.path.join(pkg_root, 'model', 'dishcolor', 'vgg_cnn_m_1024_dishcolor_dual_iter_2000.caffemodel')
        self._net = caffe.Net(self._proto_file, self._model_file, caffe.TEST)
        self._net.blobs['data'].reshape(self._test_batch_size, 3, self._patch_size, self._patch_size)


    def run_classifier(self, data):
        self._net.blobs['data'].data[:] = data
        out = self._net.forward()
        obj_prob = out['cls_prob'].astype('float32')
        color_prob = out['color_cls_prob'].astype('float32')

        prob = obj_prob[:,:,None] + color_prob[:,None,:]
        prob = prob.reshape([prob.shape[0], -1])

        return prob


