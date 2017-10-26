# Goal Metrics
![Python Version](goal/badges/python-2.7.6-blue.svg)
[![GitHub commit activity the past week, 4 weeks, yea](https://img.shields.io/github/commit-activity/4w/ipa-flg-ma/goal_metrics.svg)](https://github.com/ipa-flg-ma/goal_metrics)
[![GitHub repo size in bytes](https://img.shields.io/github/repo-size/ipa-flg-ma/goal_metrics.svg)](https://github.com/ipa-flg-ma/goal_metrics)

Writinig a python programm to get the goal data as `[x, y, R, P, Y]` and compare it to the actual position
of the robot, given as `[x, y, R, P, Y]`. Calculate the distance and angle between the robot position and the goal.
Max allowed difference can be set as parameters.

## Timing
Best timing for 2 subscribed topics in `application.py` for `goal_metrics`:
```python
    rospy.sleep(.2)
    [...]
    rospy.sleep(3)
```
Timing with `rospy.sleep()` is necessary because otherwise there will be a threading error. The bagfile includes
the topics needed for `goal_metrics`.
```yaml
topics:      /atf/testblock_nav/api          2 msgs    : atf_msgs/Api                     
             /atf/testblock_nav/trigger      2 msgs    : atf_msgs/TestblockTrigger        
             /base_pose_ground_truth      1143 msgs    : nav_msgs/Odometry                
             /move_base/goal                 1 msg     : move_base_msgs/MoveBaseActionGoal
```
detailed example is shown below.
```python
class Application:
    def __init__(self):
        rp = RvizPublisher()
        filepath = '/home/flg-ma/git/catkin_ws/src/msh/msh_bringup/launch/t_passage.launch'
        rp.main(filepath, True, False, 2.0, 0.0, 0, 0, 0)
        rospy.sleep(.2)                     # improved speed with localisation
        self.atf = ATF()

    def execute(self):
        self.atf.start("testblock_nav")
        # necessary to catch goal published on topic /move_base/goal
        rospy.sleep(3)
        sss.move("base", [4.0, 0.0, 0.0])
        self.atf.stop("testblock_nav")
        self.atf.shutdown()

```

## Error Monitoring

### `interface.subscribers.type` must be of type str
The errorlog shown below occurs often and stops `ATF` from exiting normally. Therefore no
bagfile is written and no output can be generated using the provided `metrics`.

```bash
except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))
File "/opt/ros/indigo/lib/python2.7/dist-packages/genpy/message.py", line 331, in _check_types
check_type(n, t, getattr(self, n))
File "/opt/ros/indigo/lib/python2.7/dist-packages/genpy/message.py", line 253, in check_type
check_type(field_name+"[]", base_type, v)
File "/opt/ros/indigo/lib/python2.7/dist-packages/genpy/message.py", line 263, in check_type
check_type("%s.%s"%(field_name,n), t, getattr(field_val, n))
File "/opt/ros/indigo/lib/python2.7/dist-packages/genpy/message.py", line 263, in check_type
check_type("%s.%s"%(field_name,n), t, getattr(field_val, n))
File "/opt/ros/indigo/lib/python2.7/dist-packages/genpy/message.py", line 253, in check_type
check_type(field_name+"[]", base_type, v)
File "/opt/ros/indigo/lib/python2.7/dist-packages/genpy/message.py", line 263, in check_type
check_type("%s.%s"%(field_name,n), t, getattr(field_val, n))
File "/opt/ros/indigo/lib/python2.7/dist-packages/genpy/message.py", line 229, in check_type
raise SerializationError('field %s must be of type str'%field_name)
SerializationError: field nodes[].interface.subscribers[].type must be of type str
```

The Error can be preventend when the lines below in the file `/home/flg-ma/git/catkin_ws/src/msh/msh_bringup/launch/application.xml`
are **included**! The `robot_status_retriever.py` is necessary to complete the writing of the output in a `bagfile`.

```xml
<!-- sensorring -->
<group>
    <machine name="$(arg s1)" address="$(arg s1)" env-loader="$(arg env-script)" default="true" timeout="30"/>
    <include ns="people_detection_sensorring" file="$(find cob_people_detection)/ros/launch/people_detection.launch">
        <arg name="camera_namespace" value="sensorring_cam3d_upright"/>
        <arg name="launch_head_detector" value="true"/>
        <arg name="launch_face_detector" value="true"/>
        <arg name="launch_face_recognizer" value="false"/>
        <arg name="launch_detection_tracker" value="false"/>
        <arg name="launch_face_capture" value="false"/>
        <arg name="launch_coordinator" value="false"/>
        <arg name="display_results_with_image_view" value="false"/>
    </include>
    <!--necessary, otherwise error occurs: SerializationError: field nodes[].interface.subscribers[].type must be of type str-->
    <node pkg="msh_bringup" type="robot_status_retriever.py" name="robot_status" output="screen"/>
    <include file="$(find msh_bringup)/launch/poi.launch">
        <arg name="robot" value="$(arg robot)"/>
    </include>
    <node pkg="zbar_ros" type="barcode_reader_node" name="barcode_reader" output="screen">
        <remap from="image" to="/sensorring_cam3d/rgb/image_raw"/>
    </node>
</group>
```
### `'std::out_of_range' what(): map::at`
The errorlog shown below occurs when the robot can't find a path on the global costmap.

```shell
[ WARN] [1509019721.735995968, 7.195000000]: Path Segment ps 12 has partially blocked check the costmap. Illegal point idc: 15
[ INFO] [1509019721.736094996, 7.195000000]: The blocked parts were cleared
terminate called after throwing an instance of 'std::out_of_range'
  what():  map::at
[move_base-85] process has died [pid 6139, exit code -6, cmd /opt/ros/indigo/lib/move_base/move_base cmd_vel:=/base/twist_mux/command_navigation odom:=/base/odometry_controller/odometry __name:=move_base __log:=/home/flg-ma/.ros/log/4ca1f794-ba46-11e7-ad95-782bcbad4e67/move_base-85.log].
log file: /home/flg-ma/.ros/log/4ca1f794-ba46-11e7-ad95-782bcbad4e67/move_base-85*.log
```

To fix this error, the `path_segments.yaml` has to be renewed, using `roslaunch ipa_navigation_bringup config_gui.launch pkg_env_config:=`rospack find msh_env_config` robot_env:=$ROBOT_ENV$` where `$ROBOT_ENV$` is the desired *robot_env*.
I.e.:
- `line_passage`
- `line_passage_obstacle`
- `line_passage_person_moving`
- `narrow_passage_2_cone`
- `t_passage`
- `t_passage_obstacle`
- ... (if more environments were added later...)


## History
**V 1.0.0:**
- first push

# HOW-TO New Metric
The following steps are needed to implement a new metrics in ATF:
### Python File
- Create new python-file for the metrics, using the following nameconvention:
```
calculate_*name*.py
```
- copy existing structure from one of the implemented metrics, looking like:
```python
class CalculatePublishRateParamHandler
    def parse_parameter(self, testblock_name, params):
class CalculatePublishRate:
    def __init__(self, groundtruth, groundtruth_epsilon):
    def start(self, timestamp):
    def stop(self, timestamp):  
    def pause(self, timestamp):
    def purge(self, timestamp):   
    def get_result(self):
```
  using the "publish\_rate"-metrics as an example. Replace "PublishRate" with the name of your newly generated metrics.
- In file ```atf/src/atf/atf_metrics/src/atf_metrics/__init__.py``` add:
```python
from atf_metrics.calculate_*name* import Calculate*Name*, Calculate*Name*ParamHandler
```
  e.g.
```python
from atf_metrics.calculate_jerk import CalculateJerk, CalculateJerkParamHandler
```
  here *name* stands for the name of your new metric (obviously).

- In file ```atf/src/atf/atf_metrics/config/metrics.yaml``` add:
```
*name*:
   handler: Calculate*Name*ParamHandler
```
  e.g.
```
jerk:
  handler: CalculateJerkParamHandler
```
### ATF Presenter
- In file ```atf/atf_presenter/html/js/atf_tools/test_list.js``` add (using "jerk" as an example):
```javascript
var plot_options = {
      jerk: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'Jerk'
        },
        yAxis: {
          title: {
            text: 'Jerk [m/s^3]'
          }
        },
        xAxis: {
          labels: {
            enabled: false
          }
        },
        plotOptions: {},
        tooltip: plot_tooltip
      },
};
```
  search the following if-statement:
```javascript
if ((metric_name == 'time') || (metric_name == 'path_length') || (metric_name == 'publish_rate') || (metric_name == 'interface') || (metric_name == 'jerk'))
```
  and add the new metrics as ```|| (metric_name == '*name*')```. In the following lines...
```javascript
if (metric_name == 'path_length') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['root_frame'] + " to " + metric_data['details']['measured_frame'] + ")"
if (metric_name == 'publish_rate') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['topic'] + ")"
if (metric_name == 'interface') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")"
if (metric_name == 'jerk') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['topic'] + ")"
```
  add...
```javascript
if (metric_name == '*name*') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")"
```
To get additional information in the presenter. The "details" you store in the "metrics\_data" will be shown below the metrics-name in brackets.
