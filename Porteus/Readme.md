# Readme for Porteus
Modules are on my Google Drive:
https://drive.google.com/drive/folders/0B9a0eKLI85yHNFNxNkVNWGlsdm8?usp=sharing

I have to use my own VTK and OpenCV modules. Those in repositories uses Qt4 and rtabmap uses Qt5. 
If program tries to load both versions together, it crash with SIGSEGV.

I merged all downloaded modules to one: mergedModules.xzm. It contains this packages:
addedCmakeForQt.xzm
boost-1.59.0-x86_64-1.xzm
crippled_sources.xzm
eigen3-3.2.7-x86_64-2.xzm
ffmpeg-2.8.7-x86_64_custom-2_slonly.xzm
gcc-gfortran-5.3.0-x86_64-3.xzm
gl2ps-1.3.7-x86_64-1_slonly.xzm
hdf5-1.8.15_patch1-x86_64-1_slonly.xzm
libxml2-2.9.4-x86_64-2.xzm
mychanges.xzm
netcdf-4.4.1-x86_64-3_slonly.xzm
python-2.7.11-x86_64-2.xzm
tcl-8.6.5-x86_64-2.xzm
tk-8.6.5-x86_64-2.xzm

Package addedCmakeForQt contains cmake files and include for Qt library. It is missing in current Porteus 3.2 but should be there in future.
