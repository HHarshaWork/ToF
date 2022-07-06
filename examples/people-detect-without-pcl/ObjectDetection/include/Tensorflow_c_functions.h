/******************************************************************************
Copyright(c) 2016 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef TENSORFLOW_C_FUN_H
#define TENSORFLOW_C_FUN_H

#include <tensorflow/c/c_api.h>
#include <memory>
#include <algorithm>
#include <cstddef>
#include <iterator>
#include <vector>
#include <assert.h>
#include <string.h>
#include <stdint.h>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

/**
* A Wrapper for the C API status object.
*/
class CStatus {
public:
    TF_Status* ptr;
    CStatus() {
        ptr = TF_NewStatus();
    }

    /**
        * Dump the current error message.
        */
    void dump_error()const {
        std::cerr << "TF status error: " << TF_Message(ptr) << std::endl;
    }

    /**
        * Return a boolean indicating whether there was a failure condition.
        * @return
        */
    inline bool failure()const {
        return TF_GetCode(ptr) != TF_OK;
    }

    ~CStatus() {
        if (ptr)TF_DeleteStatus(ptr);
    }
};


namespace detail {
    template<class T>
    class TFObjDeallocator;

    template<>
    struct TFObjDeallocator<TF_Status> { static void run(TF_Status* obj) { TF_DeleteStatus(obj); } };

    template<>
    struct TFObjDeallocator<TF_Graph> { static void run(TF_Graph* obj) { TF_DeleteGraph(obj); } };

    template<>
    struct TFObjDeallocator<TF_Tensor> { static void run(TF_Tensor* obj) { TF_DeleteTensor(obj); } };

    template<>
    struct TFObjDeallocator<TF_SessionOptions> { static void run(TF_SessionOptions* obj) { TF_DeleteSessionOptions(obj); } };

    template<>
    struct TFObjDeallocator<TF_Buffer> { static void run(TF_Buffer* obj) { TF_DeleteBuffer(obj); } };

    template<>
    struct TFObjDeallocator<TF_ImportGraphDefOptions> {
        static void run(TF_ImportGraphDefOptions* obj) { TF_DeleteImportGraphDefOptions(obj); }
    };

    template<>
    struct TFObjDeallocator<TF_Session> {
        static void run(TF_Session* obj) {
            CStatus status;
            TF_DeleteSession(obj, status.ptr);
            if (status.failure()) {
                status.dump_error();
            }
        }
    };
}

template<class T> struct TFObjDeleter {
    void operator()(T* ptr) const {
        detail::TFObjDeallocator<T>::run(ptr);
    }
};

template<class T> struct TFObjMeta {
    typedef std::unique_ptr<T, TFObjDeleter<T>> UniquePtr;
};


#define MY_TENSOR_SHAPE_MAX_DIM 16
struct TensorShape {
    int64_t values[MY_TENSOR_SHAPE_MAX_DIM];
    int dim;

    int64_t size()const {
        assert(dim >= 0);
        int64_t v = 1;
        for (int i = 0; i < dim; i++)v *= values[i];
        return v;
    }
};

class MySession {
public:
    typename TFObjMeta<TF_Graph>::UniquePtr graph;
    typename TFObjMeta<TF_Session>::UniquePtr session;

    TF_Output inputs, outputs[3];
};

MySession* my_model_load(const char* filename, const char* input_name,
    const char* output_name1, const char* output_name2, const char* output_name3);
TF_Tensor* img2tensor(const cv::Mat img, const TensorShape& shape);

template<class T> typename TFObjMeta<T>::UniquePtr tf_obj_unique_ptr(T* obj) {
    typename TFObjMeta<T>::UniquePtr ptr(obj);
    return ptr;
}
#endif /* TENSORFLOW_C_FUN_H */
