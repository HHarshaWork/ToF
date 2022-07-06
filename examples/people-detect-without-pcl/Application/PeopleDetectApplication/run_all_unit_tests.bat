::test different display modes 
::cm 10 mp
::.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 10 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_cm_10.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_cm_10.csv -cm 10 
::cm 7 qmp 
::.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 10 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_cm_10.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_cm_7.csv -cm 7

::test different output\unit_tests\ modes
::ir
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_dm_0.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_dm_0.csv -dm 0 
::depth 
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_dm_1.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_dm_1.csv -dm 1 
::sil
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_dm_2.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_dm_2.csv -dm 2 

::test num of frames
::1
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 1 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_n_1.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_n_1.csv 
::10 
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 10 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_n_10.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_n_10.csv 
::100
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_n_100.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_n_100.csv  

::test different height offsets
::0
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_off_0.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_off_0.csv -off 0
::10 
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_off_10.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_off_10.csv -off 10
::100
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_off_100.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_off_100.csv -off 100
::10000000
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_off_10000000.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_off_10000000.csv -off 10000000 


::test different output\unit_tests\ file modes 
::both csv and avi files
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 10 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_both_csv_metadata.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_both_csv_metadata.csv 
::only metadata csv
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 10 -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_only_csv.csv 
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 10 -em 1
::only avi
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_RAW_off_100.avi -em 0 


::test different tracking times  
::0
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_th_0.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_th_0.csv -th 0 
::10
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_th_10.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_th_10.csv -th 10 
::20
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_th_20.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_th_20.csv -th 20 
::90
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 300 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_th_90.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_th_90.csv -th 90 

::test different distance in cm
::0
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_d_0.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_d_0.csv -d 0 
::10
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_d_10.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_d_10.csv -d 10 
::20
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 100 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_d_20.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_d_20.csv -d 20 
::90
.\Bin\Release\PeopleDetectApplication.exe -i ..\..\Utils\example_data\3m_sitting_apart_RAW_Combined_Depth_AB.bin -n 300 -o ..\..\Utils\output\unit_tests\\3m_sitting_apart_d_90.avi -em 1 -md ..\..\Utils\output\unit_tests\\3m_sitting_apart_th_90.csv -d 90 