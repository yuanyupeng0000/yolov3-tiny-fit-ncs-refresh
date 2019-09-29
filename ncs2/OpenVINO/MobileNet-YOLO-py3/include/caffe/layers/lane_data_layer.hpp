#ifndef CAFFE_DATA_LAYER_HPP_
#define CAFFE_DATA_LAYER_HPP_

#include <string>
#include <vector>

#include "caffe/blob.hpp"
#include "caffe/data_reader.hpp"
#include "caffe/data_transformer.hpp"
#include "caffe/internal_thread.hpp"
#include "caffe/layer.hpp"
#include "caffe/layers/base_data_layer.hpp"
#include "caffe/proto/caffe.pb.h"
#include "caffe/util/db.hpp"

namespace caffe {

template <typename Dtype>
class LaneDataLayer : public BasePrefetchingDataLayer<Dtype> {
 public:
  explicit LaneDataLayer(const LayerParameter& param);
  virtual ~LaneDataLayer();
  virtual void DataLayerSetUp(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);
  // AnnotatedDataLayer uses DataReader instead for sharing for parallelism
  virtual inline bool ShareInParallel() const { return false; }
  virtual inline const char* type() const { return "LaneData"; }
  virtual inline int ExactNumBottomBlobs() const { return 0; }
  virtual inline int MinTopBlobs() const { return 1; }

 protected:
  virtual void load_batch(Batch<Dtype>* batch);

  DataReader<AnnotatedDatum> reader_;
  bool has_anno_type_;
  AnnotatedDatum_AnnotationType anno_type_;
  vector<BatchSampler> batch_samplers_;
  string label_map_file_;
  int yolo_data_type_;
  float yolo_data_jitter_;
  bool train_diffcult_;
  int iters_;
  int policy_num_ ;
  bool single_class_; //for yolo segementation
  YoloSegLabel label_map_;
  int seg_label_maxima_;
  int seg_scales_;
  int seg_resize_width_;
  int seg_resize_height_;
};

}  // namespace caffe

#endif  // CAFFE_DATA_LAYER_HPP_
