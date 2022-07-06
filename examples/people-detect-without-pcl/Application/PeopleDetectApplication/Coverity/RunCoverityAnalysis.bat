echo off
:: Del intermediate files
IF NOT EXIST int-dir GOTO BUILD
rd /s /q int-dir

:BUILD
:: remove previous build directories 
rd /s /q ..\x64
rd /s /q ..\..\..\ObjectDetection\ObjectDetector\x64
rd /s /q ..\..\..\PeopleDetection\PeopleDetectionSDK\x64


C:\work\cov-analysis-win64-2021.06\bin\cov-build --dir int-dir --config C:\work\cov-analysis-win64-2021.06\cov-config\coverity_config.xml build.bat
if ERRORLEVEL 1 goto failed

C:\work\cov-analysis-win64-2021.06\bin\cov-analyze --dir int-dir --config C:\work\cov-analysis-win64-2021.06\cov-config\coverity_config.xml --security --concurrency --enable-constraint-fpp --enable-fnptr --enable-virtual --enable-callgraph-metrics -j auto --strip-path "C:\work\deeplearning_windowsdk\SDK\" > coverity_analyze.txt 2>&1

C:\work\cov-analysis-win64-2021.06\bin\cov-format-errors --dir int-dir --html-output PplTrackerReport

::Search for errors
findstr /c:"Defect occurrences found       : 0" coverity_analyze.txt

if errorlevel 1 goto failed

::coverity_analyze.txt
:success
echo "Done"
exit /B 0
:failed
echo "failed" %ERRORLEVEL%
exit /B 1
