

## 3.1.5.3. 3FG15/3FG25

The table below provides an overview of the available MODBUS registers in the 3FG.
All writable registers can be accessed using function codes 6, 16 or 23 and all readable
registers can be accessed using function codes 3 or 23.

Address         Register                        Access
0       0x0000  Target force                    Write
1       0x0001  Target diameter                 Write
2       0x0002  Grip type                       Write
3       0x0003  Control                         Write
256     0x0100  Status                          Read only
257     0x0101  Raw diameter                    Read only
258     0x0102  Diameter with fingertip offset  Read only
259     0x0103  Force applied                   Read only
270     0x010E  Finger length                   Read only
272     0x0110  Finger position                 Read only
273     0x0111  Fingertip offset                Read only
275     0x0113  Actual width with offset        Read only
513     0x0201  Minimum diameter                Read only
514     0x0202  Maximum diameter                Read only
1025    0x0401  Set finger length               Read/Write
1027    0x0403  Set finger position             Read/Write
1028    0x0404  Set fingertip offset            Read/Write

#### 0 (0x0000) Target force (Write)
This field sets the target force to be reached when gripping and holding a workpiece. It must
be provided in 10*%. The valid range is 0 to 1000.

#### 1 (0x0001) Target diameter (Write)
This field sets the target diameter to achieve. It must be provided in 1/10th millimeters. The
valid range depends on the finger position, finger length and fingertip diameter. For more
information see the Technical sheet section.

#### 2 (0x0002) Grip type (Write)
This field sets whether the grip will be external 0 or internal 1. It also sets the if the diameter is
measured from the inside of the fingertips (external grip) or from the outside of the fingertips
(internal grip).

#### 3 (0x0003) Control (Write)
The control field is used to start and stop gripper motion. Only one option should be set at a
time. Please note that the gripper will not start a new motion before the one currently being
executed is done (see busy flag in the Status field). The valid commands are:
Value       Name            Description
1 (0x0001)  grip            Start the motion, with the preset target force and diameter.
                            Please note that the gripper will ignore this command if the
                            busy flag is set in the status field.
2 (0x0002)  move            Start the motion without applying the target force
4 (0x0004)  stop            Stop the current motion.
5 (0x0005)  flexible grip   The fingers will move from the current diameter towards the
                            target diameter, and do a grip with the desired force. Maximum
                            force is 140 N when 100% is selected, and payload is maximum
                            8 kg.

#### 256 (0x0100) Status (Read only)
This status field indicates the status of the gripper and its motion. It is composed of 7 flags,
described in the table below.
Bit     Name                Description
0 (LSB) busy                High (1) when a motion is ongoing, low (0) when not. The
                            gripper will only accept new commands when this flag is low.
1       grip detected       High (1) when an internal- or external grip is detected.
2       Force grip detected High (1) when an internal- or external grip with the target
                            force is detected.
3       calibration         Whether calibration is OK or not.
4-16    Reserved            Not used

#### 257 (0x0101) Raw diameter (Read only)
Indicates the current diameter measured from the center of the fingertips.

#### 258 (0x0102) Diameter with fingertip offset (Read only)
Indicates the current diameter considering the fingertip offset in 1/10 millimeters. Please note
that the value is a signed two’s complement number.

#### 259 (0x0103) Force applied (Read only)
Indicates the force applied in 1/10 %.

#### 270 (0x010E) Finger length (Read only)
Indicates the length of the finger in 1/10 mm

#### 272 (0x0110) Finger position (Read only)
Indicates how the finger is mounted. Positions available are 1, 2 and 3.

#### 273 (0x0111) Fingertip offset (Read only)
This field sets the Fingertip offset in 1/100 mm.

#### 275 (0x0113) Actual width with offset (Read only)
Indicates the current width between the gripper fingers in 1/10 millimeters. The set fingertip
offset is considered.

#### 513 (0x0201) Minimum diameter (Read only)
Indicates the minimum reachable diameter depending on the finger position, finger length
and fingertip diameter. For more information see the
Technical sheet section.

#### 514 (0x0202) Maximum diameter (Read only)
Indicates the maximum reachable diameter depending on the finger position, finger length
and fingertip diameter. For more information see the
Technical sheet section.

#### 1025 (0x0401) Set Finger length (Read/Write)
This field sets the finger length in 1/10 mm.

#### 1027 (0x0403) Set Finger position (Read/Write)
This field sets the finger position 1, 2 or 3.

#### 1028 (0x0404) Set Fingertip offset (Read/Write)
This field sets the fingertip offset diameter in 1/100 mm.
