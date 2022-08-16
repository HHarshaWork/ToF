::revert all unchanged files first 
git checkout .

cd Documents\Doxygen
doxygen people_det_doxyfile
cd ..\..

::Build the solution to ensure that it builds fine 
"C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\MSBuild.exe" -t:rebuild -property:Configuration=Release Application\PeopleDetectApplication\PeopleDetectApplication.sln > BuildLog.txt

::Search for errors
findstr /c:"0 Error(s)" BuildLog.txt

if errorlevel 1 goto failed

::delete dlls before packaging 
::copy Application\PeopleDetectApplication\Bin\Debug\aditof.dll Application\PeopleDetectApplication\Bin\
::del /q Application\PeopleDetectApplication\Bin\Debug\*
::copy Application\PeopleDetectApplication\Bin\aditof.dll Application\PeopleDetectApplication\Bin\Debug\
::copy Application\PeopleDetectApplication\Bin\Release\aditof.dll Application\PeopleDetectApplication\Bin\
::del /q Application\PeopleDetectApplication\Bin\Release\*
::copy Application\PeopleDetectApplication\Bin\aditof.dll Application\PeopleDetectApplication\Bin\Release\
::del /q Dependencies\glog\include\glog\*
::del /q Dependencies\glog\include\*
::del /q Dependencies\glog\lib\Debug\*
::del /q Dependencies\glog\lib\Release\*
::del /q Dependencies\opencv\include\*
::del /q Dependencies\opencv\lib\Debug\*
::del /q Dependencies\opencv\lib\Release\*
::del /q Utils\output\*

::This runs the packager script to generate the zip. Packager is a whl file that can be got from https://www.secad.analog.com/jenkins/cse/job/Packager_Build_Test/6/.
:: Maintained by Conor Pesch
:success
python.exe -m packager -m manifests -p %cd% -n 3DSensorPeopleTracker -v 1.0.2
echo "Packaging Done"
exit /B 0
:failed
echo "Packaging failed" %ERRORLEVEL%
exit /B 1