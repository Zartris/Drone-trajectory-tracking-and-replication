##Calibration board information:
https://github.com/ethz-asl/kalibr/wiki/calibration-targets

* Yaml is the config
* April_6x6 is the scaled and unscaled version

```yaml
target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.024           #size of apriltag, edge to edge [m]
tagSpacing: 0.029166667  #ratio of space between tags to tagSize
                         #example: tagSize=0.24m, spacing=0.007m --> tagSpacing=0.25[-]
```



![kalibr](https://user-images.githubusercontent.com/5337083/41458381-be379c6e-7086-11e8-9291-352445140e88.png)





To convert image_raw to image_mono:

```
ROS_NAMESPACE=bebop rosrun image_proc image_proc
rosrun image_view image_view image:=/bebop/image_mono
```