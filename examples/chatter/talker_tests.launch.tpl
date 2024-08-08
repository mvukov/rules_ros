<launch>
  <node name="talker" type="chatter/talker" />

  <test name="advertisetest"
      test-name="advertisetest"
      type="{advertisetest}">
    <rosparam>
      topics:
        - name: /chatter
          timeout: 0.1
          type: example_msgs/Example
    </rosparam>
  </test>

  <test name="publishtest"
        test-name="publishtest"
        type="{publishtest}">
    <rosparam>
      topics:
        - name: /chatter
          timeout: 0.1
          negative: False
    </rosparam>
  </test>
</launch>
