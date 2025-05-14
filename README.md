# robosoccer_control
Translates motion plans into actual joint movements

### Publishes
* `JointState`: `/joint_cmd`: Robot limb joint commands

### Subscribes to
* `String`: `/footsteps`: Position to place foot down
* `PoseStamped`: `/target_pose`: Desired position of robot

