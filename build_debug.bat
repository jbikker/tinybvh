REM This batch file produces tiny_bvh_benchmark.exe and tiny_bvh_benchmark.pdb,
REM ready for profiling using Superluminal.

g++ -std=c++20 -mavx2 -mfma -gdwarf-2 -Ofast tiny_bvh_benchmark.cpp benchmark/acc_struc.cpp benchmark/experiment.cpp benchmark/primitive_set.cpp benchmark/ray_distribution.cpp benchmark/tiny_bvh.cpp benchmark/tools.cpp -Iexternal/embree/include -Iexternal/madmann91 -Iexternal/OpenCL/inc -Ibenchmark -I. -o tiny_bvh_benchmark -Lexternal/embree/lib -Lexternal/OpenCL/lib -l embree4 -l tbb -l OpenCL
cv2pdb tiny_bvh_benchmark.exe
