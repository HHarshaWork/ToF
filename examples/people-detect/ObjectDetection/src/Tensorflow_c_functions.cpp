#include <Windows.h>
#include "../ObjectDetector/resource.h"

#include "tensorflow_c_functions.h"

using namespace std;

/**
* Deallocator for TF_Buffer data.
* @tparam T
* @param data
* @param length
*/
template<class T> static void free_cpp_array(void* data, size_t length) {
    delete[]((T*)data);
}

/**
* Deallocator for TF_NewTensor data.
* @tparam T
* @param data
* @param length
* @param arg
*/
template<class T> static void cpp_array_deallocator(void* data, size_t length, void* arg) {
    delete[]((T*)data);
}


TF_Tensor* img2tensor(const cv::Mat img, const TensorShape& shape) {
    auto size = img.size();
    auto total_size = size.height * size.width * 3;
    auto output_array = std::make_unique<float[]>(total_size);
    //make_unique fn constructs an obj of type float(here) and wraps it in a std::unique_ptr 
    //std::unique_ptr is a smart pointer that owns and manages another object through a pointer and disposes of that object when the unique_ptr goes out of scope.
    float* dst_ptr = output_array.get(); //std::unique_ptr::get -> Returns the stored pointer.
    float* ptr = (float*)img.data;
    memcpy(dst_ptr, ptr, total_size * sizeof(float));
    auto output = tf_obj_unique_ptr(TF_NewTensor(TF_FLOAT, shape.values, shape.dim, (void*)output_array.get(), total_size * sizeof(float), cpp_array_deallocator<float>, nullptr));
    //std::cout << typeid(output).name() << std::endl;
    if (output) {
        // The ownership has been successfully transferred
        output_array.release(); //std::unique_ptr.release(); -> Releases ownership of its stored pointer, by returning its value and replacing it with a null pointer. Returns pointer to the object managed by unique_ptr before the call.
    }
    return output.release();
}


/**
 * Read the entire content of a file and return it as a TF_Buffer.
 * @param file: The file to be loaded.
 * @return
 */
static TF_Buffer* read_tf_buffer_from_file(const char* file) {
    std::ifstream t(file, std::ifstream::binary);
    t.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    t.seekg(0, std::ios::end);
    size_t size = t.tellg();
    auto data = std::make_unique<char[]>(size);
    t.seekg(0);
    t.read(data.get(), size);

    TF_Buffer* buf = TF_NewBuffer();
    buf->data = data.release();
    buf->length = size;
    buf->data_deallocator = free_cpp_array<char>;
    return buf;
}

// near the top of your CPP file
EXTERN_C IMAGE_DOS_HEADER __ImageBase;

static TF_Buffer* read_tf_buffer_from_buffer() {

    LPTSTR  strDLLPath1 = new TCHAR[_MAX_PATH];
    ::GetModuleFileName((HINSTANCE)&__ImageBase, strDLLPath1, _MAX_PATH);
    //IS_INTRESOURCE(IDR_RCDATA1);

    HRSRC hResource = FindResource((HINSTANCE)&__ImageBase, MAKEINTRESOURCE(IDR_RCDATA1), RT_RCDATA);
    HGLOBAL hMemory = LoadResource((HINSTANCE)&__ImageBase, hResource);
    DWORD dwSize = SizeofResource((HINSTANCE)&__ImageBase, hResource);
    LPVOID lpAddress = LockResource(hMemory);

    char* bytes = new char[dwSize];
    memcpy(bytes, lpAddress, dwSize);    

    TF_Buffer* buf = TF_NewBuffer();
    buf->data = bytes;
    buf->length = dwSize;
    buf->data_deallocator = free_cpp_array<char>;

    delete[] strDLLPath1;
    return buf;
}

/**
* Load a GraphDef from a provided file.
* @param filename: The file containing the protobuf encoded GraphDef
* @param input_name: The name of the input placeholder
* @param output_name: The name of the output tensor
* @return
*/
MySession* my_model_load(const char* filename, const char* input_name,
    const char* output_name1, const char* output_name2, const char* output_name3) {
    CStatus status;

    auto graph = tf_obj_unique_ptr(TF_NewGraph());
    {
        // Load a protobuf containing a GraphDef
        //auto graph_def = tf_obj_unique_ptr(read_tf_buffer_from_file(filename));        
        auto graph_def = tf_obj_unique_ptr(read_tf_buffer_from_buffer());        
        if (!graph_def) {
            return nullptr;
        }

        auto graph_opts = tf_obj_unique_ptr(TF_NewImportGraphDefOptions());
        TF_GraphImportGraphDef(graph.get(), graph_def.get(), graph_opts.get(), status.ptr);
    }

    if (status.failure()) {
        status.dump_error();
        return nullptr;
    }

    auto input_op = TF_GraphOperationByName(graph.get(), input_name);
    auto output_op1 = TF_GraphOperationByName(graph.get(), output_name1);
    auto output_op2 = TF_GraphOperationByName(graph.get(), output_name2);
    auto output_op3 = TF_GraphOperationByName(graph.get(), output_name3);
    if (!input_op || !output_op1 || !output_op2 || !output_op3) {
        return nullptr;
    }

    auto session = std::make_unique<MySession>();
    {
        auto opts = tf_obj_unique_ptr(TF_NewSessionOptions());
        session->session = tf_obj_unique_ptr(TF_NewSession(graph.get(), opts.get(), status.ptr));
    }

    if (status.failure()) {
        return nullptr;
    }
    assert(session);

    graph.swap(session->graph);
    session->inputs = { input_op, 0 };
    session->outputs[0] = { output_op1,0 };
    session->outputs[1] = { output_op2,0 };
    session->outputs[2] = { output_op3,0 };

    return session.release();
}
