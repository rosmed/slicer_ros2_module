Steps to create a full release.  We should be able to write a script to do this.

* Build Slicer with system SSL libraries and BUILD_TYPE set to Release.  Make sure you checkout the version you plan to release
* Build SlicerROS2 in ros2 workspace.  Make sure BUILD_TYPE is also Release

Create the initial `.tgz` for Slicer using `make package` in the directory `Slicer-build`.

The following steps are to add the ROS2 related file to distribution
and also strip the libraries.  Note that the latest SlicerROS2 build
copies its files in the Slicer-build directory but they are not added
automatically to the `.tgz`.

```sh
tar zxf Slicer-5.6.2-2024-04-05-linux-amd64.tar.gz // version and date will change
cp vtkSlicerROS2Module* Slicer-5.6.2-2024-04-05-linux-amd64 // copy .txt files
cp lib/Slicer-5.6/qt-loadable-modules/*ROS2* Slicer-5.6.2-2024-04-05-linux-amd64/lib/Slicer-5.6/qt-loadable-modules/ // copy modules
// strip and remove some old files
find Slicer-5.6.2-2024-04-05-linux-amd64 -name "*.so" -exec strip {} \;
find Slicer-5.6.2-2024-04-05-linux-amd64 -name "*.pyc" -exec rm {} \;

// rename the directory, tar and compress
mv Slicer-5.6.2-2024-04-05-linux-amd64 SlicerROS2-5.6.2-0.9-Ubuntu-20.04-Galactic-amd64
tar cf SlicerROS2-5.6.2-0.9-Ubuntu-20.04-Galactic-amd64.tar SlicerROS2-5.6.2-0.9-Ubuntu-20.04-Galactic-amd64 
gzip --best SlicerROS2-5.6.2-0.9-Ubuntu-20.04-Galactic-amd64.tar 
```

After uploading the new `.tgz`, clean using:
```sh
rm *.tgz
rm -rf SlicerROS*
```
