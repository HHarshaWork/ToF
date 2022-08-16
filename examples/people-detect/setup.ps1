#Download tensorflow dlls and copy
# Source file location
$source = 'https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-cpu-windows-x86_64-1.15.0.zip'
# Destination to save the file
$destination = 'tensorflow.zip'
#Download the file
Invoke-WebRequest -Uri $source -OutFile $destination

#unzip everything
Expand-Archive 'tensorflow.zip'

#tensorflow dlls 
Copy-Item -Path "tensorflow\lib\tensorflow.dll" -Destination "Bin\"


#Download opencv dlls 
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
# Source file location
$source = 'https://github.com/opencv/opencv/releases/download/4.5.3/opencv-4.5.3-openvino-dldt-2021.4-vc16-avx2.zip'
# Destination to save the file
$destination = 'opencv.zip'
#Download the file
Invoke-WebRequest -Uri $source -OutFile $destination

#unzip 
Expand-Archive 'opencv.zip'


#opencv copy dlls 
Copy-Item -Path "opencv\opencv\build\bin\*" -Destination "Bin\"

