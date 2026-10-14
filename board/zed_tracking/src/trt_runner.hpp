// TensorRT 10 引擎封装:加载、绑定枚举、单输入单输出的同步推理。
// 输入假定 NCHW float32(trtexec/ultralytics 导出的标准形态),输出 float32。

#pragma once

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace zed_tracking
{

class TrtLogger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::fprintf(stderr, "[TRT] %s\n", message);
    }
  }
};

inline void cuda_check(cudaError_t error, const char * what)
{
  if (error != cudaSuccess) {
    throw std::runtime_error(
      std::string(what) + " failed: " + cudaGetErrorString(error));
  }
}

class TrtRunner
{
public:
  TrtRunner() = default;
  ~TrtRunner() { release(); }
  TrtRunner(const TrtRunner &) = delete;
  TrtRunner & operator=(const TrtRunner &) = delete;

  void load(const std::string & engine_path)
  {
    std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
    if (!file) {
      throw std::runtime_error("cannot open engine file: " + engine_path);
    }
    const std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> blob(static_cast<std::size_t>(size));
    if (!file.read(blob.data(), size)) {
      throw std::runtime_error("cannot read engine file: " + engine_path);
    }
    release();
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    // CPU input is copied before enqueue; this runner shares no SDK GPU buffers.
    // Avoid implicit waits on the ZED SDK's legacy default stream.
    cuda_check(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreate");
    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
    if (engine_ == nullptr) {
      throw std::runtime_error("deserializeCudaEngine failed for " + engine_path);
    }
    context_.reset(engine_->createExecutionContext());
    if (context_ == nullptr) {
      throw std::runtime_error("createExecutionContext failed");
    }
    bind_tensors();
    allocate();
  }

  // 输入 shape 为 {1, 3, H, W}。
  const std::vector<std::int64_t> & input_shape() const { return input_shape_; }
  int input_height() const { return static_cast<int>(input_shape_[2]); }
  int input_width() const { return static_cast<int>(input_shape_[3]); }
  const std::vector<std::int64_t> & output_shape() const { return output_shape_; }
  const std::string & input_name() const { return input_name_; }
  const std::string & output_name() const { return output_name_; }

  // host_input 长度必须等于输入元素数;推理结果写回 host_output 并返回其引用。
  const std::vector<float> & infer(const float * host_input)
  {
    cuda_check(
      cudaMemcpyAsync(
        input_device_.get(), host_input, input_bytes_, cudaMemcpyHostToDevice,
        stream_),
      "input H2D");
    context_->setTensorAddress(input_name_.c_str(), input_device_.get());
    context_->setTensorAddress(output_name_.c_str(), output_device_.get());
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("enqueueV3 failed");
    }
    cuda_check(
      cudaMemcpyAsync(
        host_output_.data(), output_device_.get(), output_bytes_,
        cudaMemcpyDeviceToHost, stream_),
      "output D2H");
    cuda_check(cudaStreamSynchronize(stream_), "stream sync");
    return host_output_;
  }

private:
  struct CudaDeleter
  {
    void operator()(void * pointer) const noexcept
    {
      if (pointer != nullptr) {
        cudaFree(pointer);
      }
    }
  };

  static void delete_runtime(nvinfer1::IRuntime * pointer) noexcept
  {
    delete pointer;
  }
  static void delete_engine(nvinfer1::ICudaEngine * pointer) noexcept
  {
    delete pointer;
  }
  static void delete_context(nvinfer1::IExecutionContext * pointer) noexcept
  {
    delete pointer;
  }

  void release() noexcept
  {
    input_device_.reset();
    output_device_.reset();
    context_.reset();
    engine_.reset();
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
    runtime_.reset();
  }

  void bind_tensors()
  {
    const int count = engine_->getNbIOTensors();
    for (int index = 0; index < count; ++index) {
      const char * name = engine_->getIOTensorName(index);
      const auto mode = engine_->getTensorIOMode(name);
      auto shape = engine_->getTensorShape(name);
      const auto dtype = engine_->getTensorDataType(name);
      if (dtype != nvinfer1::DataType::kFLOAT) {
        throw std::runtime_error(std::string("tensor ") + name + " is not float32");
      }
      for (int axis = 0; axis < shape.nbDims; ++axis) {
        if (shape.d[axis] <= 0) {
          throw std::runtime_error(
            std::string("dynamic shape not supported for tensor ") + name);
        }
      }
      std::vector<std::int64_t> dims(shape.d, shape.d + shape.nbDims);
      if (mode == nvinfer1::TensorIOMode::kINPUT) {
        if (!input_name_.empty()) {
          throw std::runtime_error("multiple input tensors are not supported");
        }
        input_name_ = name;
        input_shape_ = dims;
      } else {
        if (!output_name_.empty()) {
          throw std::runtime_error("multiple output tensors are not supported");
        }
        output_name_ = name;
        output_shape_ = dims;
      }
    }
    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error("engine must expose one input and one output");
    }
    if (input_shape_.size() != 4 || input_shape_[0] != 1 || input_shape_[1] != 3) {
      throw std::runtime_error("input must be static [1,3,H,W]");
    }
  }

  std::size_t element_count(const std::vector<std::int64_t> & dims) const
  {
    std::size_t count = 1;
    for (const auto dim : dims) {
      count *= static_cast<std::size_t>(dim);
    }
    return count;
  }

  void allocate()
  {
    input_bytes_ = element_count(input_shape_) * sizeof(float);
    output_bytes_ = element_count(output_shape_) * sizeof(float);
    void * input_pointer = nullptr;
    void * output_pointer = nullptr;
    cuda_check(cudaMalloc(&input_pointer, input_bytes_), "cudaMalloc input");
    cuda_check(cudaMalloc(&output_pointer, output_bytes_), "cudaMalloc output");
    input_device_.reset(input_pointer);
    output_device_.reset(output_pointer);
    host_output_.resize(element_count(output_shape_));
  }

  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime, void (*)(nvinfer1::IRuntime *)> runtime_{
    nullptr, &delete_runtime};
  std::unique_ptr<nvinfer1::ICudaEngine, void (*)(nvinfer1::ICudaEngine *)> engine_{
    nullptr, &delete_engine};
  std::unique_ptr<
    nvinfer1::IExecutionContext, void (*)(nvinfer1::IExecutionContext *)>
  context_{nullptr, &delete_context};
  cudaStream_t stream_{nullptr};
  std::unique_ptr<void, CudaDeleter> input_device_;
  std::unique_ptr<void, CudaDeleter> output_device_;
  std::string input_name_;
  std::string output_name_;
  std::vector<std::int64_t> input_shape_;
  std::vector<std::int64_t> output_shape_;
  std::size_t input_bytes_{0};
  std::size_t output_bytes_{0};
  std::vector<float> host_output_;
};

}  // namespace zed_tracking
