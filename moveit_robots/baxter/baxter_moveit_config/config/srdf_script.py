aa = ["distal_1", "distal_2", "distal_3", "distal_pad_1", "distal_pad_2", "distal_pad_3", "finger_1_/flex_link_1_", "finger_1_/flex_link_2_", "finger_1_/flex_link_3_", "finger_1_/flex_link_4_", "finger_1_/flex_link_5_", "finger_1_/flex_link_6_", "finger_1_/flex_link_7_", "finger_1_/flex_link_8_", "finger_1_/flex_link_9_", "finger_1_/flex_link_1_", "finger_1_/flex_link_2_", "finger_1_/flex_link_3_", "finger_1_/flex_link_4_",
"finger_1_/flex_link_5_", "finger_1_/flex_link_6_", "finger_1_/flex_link_7_", "finger_1_/flex_link_8_", "finger_1_/flex_link_9_", "finger_1_/flex_link_1_", "finger_1_/flex_link_2_", "finger_1_/flex_link_3_", "finger_1_/flex_link_4_", "finger_1_/flex_link_5_", "finger_1_/flex_link_6_", "finger_1_/flex_link_7_", "finger_1_/flex_link_8_", "finger_1_/flex_link_9_", "left_gripper_base", "pad", "proximal_1", "proximal_2", "proximal_3", "proximal_pad_1",
"proximal_pad_2", "proximal_pad_3", "swivel_1", "swivel_2"]

f = open('file', "w")

for i in range(0,len(aa)):
    for j in range(i+1,len(aa)):
        msg = '<disable_collisions link1="' + aa[i] + '"      link2="' + aa[j] + '" reason="Never" />\n'
        f.write(msg)
