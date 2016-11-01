tags:
  - ""
  - ""
  - debug
  - regression
description: "8 Sources send messages to one Slot\n"
name: Parallel Slot Tester
yaml:
  connections:
    - targets:
        - csapex::Delay_1:|:slot_0
      uuid: csapex::ExampleImageGenerator_3:|:out_0
      types:
        - default
    - types:
        - default
      targets:
        - csapex::Delay_1:|:slot_0
      uuid: csapex::ExampleImageGenerator_0:|:out_0
    - targets:
        - csapex::Delay_1:|:slot_0
      uuid: csapex::ExampleImageGenerator_11:|:out_0
      types:
        - default
    - uuid: csapex::ExampleImageGenerator_9:|:out_0
      targets:
        - csapex::Delay_1:|:slot_0
      types:
        - default
    - uuid: csapex::ExampleImageGenerator_8:|:out_0
      targets:
        - csapex::Delay_1:|:slot_0
      types:
        - default
    - uuid: csapex::ExampleImageGenerator_1:|:out_0
      targets:
        - csapex::Delay_1:|:slot_0
      types:
        - default
    - uuid: csapex::ExampleImageGenerator_2:|:out_0
      targets:
        - csapex::Delay_1:|:slot_0
      types:
        - default
    - types:
        - default
      uuid: csapex::ExampleImageGenerator_10:|:out_0
      targets:
        - csapex::Delay_1:|:slot_0
  nodes:
    - state:
        params:
          {}
        persistent_params:
          []
      z: 4
      logger_level: 1
      exec_mode: 1
      enabled: true
      minimized: false
      flipped: false
      muted: false
      pos:
        - -637
        - -144
      label: csapex::ExampleImageGenerator_0
      uuid: csapex::ExampleImageGenerator_10
      color:
        - -1
        - -1
        - -1
      type: csapex::ExampleImageGenerator
    - logger_level: 1
      state:
        persistent_params:
          []
        params:
          {}
      flipped: false
      exec_mode: 1
      muted: false
      label: csapex::ExampleImageGenerator_0
      enabled: true
      type: csapex::ExampleImageGenerator
      pos:
        - -617
        - -63
      color:
        - -1
        - -1
        - -1
      uuid: csapex::ExampleImageGenerator_2
      minimized: false
      z: 6
    - type: csapex::Delay
      uuid: csapex::Delay_1
      pos:
        - -260
        - 305
      state:
        params:
          delay progress:
            name: delay progress
            type: progress
          delay:
            step: 0.01
            type: range
            double: 0
            name: delay
            min: 0
            max: 10
          blocking:
            name: blocking
            bool: false
            type: value
        persistent_params:
          []
      label: csapex::Delay_0
      logger_level: 1
      enabled: true
      flipped: false
      exec_mode: 1
      muted: false
      minimized: false
      z: 17
      color:
        - -1
        - -1
        - -1
    - logger_level: 1
      state:
        params:
          {}
        persistent_params:
          []
      color:
        - -1
        - -1
        - -1
      uuid: csapex::ExampleImageGenerator_1
      type: csapex::ExampleImageGenerator
      pos:
        - -2
        - -246
      z: 5
      exec_mode: 1
      enabled: true
      flipped: false
      label: csapex::ExampleImageGenerator_0
      muted: false
      minimized: false
    - z: 5
      muted: false
      uuid: csapex::ExampleImageGenerator_8
      type: csapex::ExampleImageGenerator
      color:
        - -1
        - -1
        - -1
      pos:
        - 12
        - -127
      enabled: true
      flipped: false
      minimized: false
      exec_mode: 1
      logger_level: 1
      state:
        params:
          {}
        persistent_params:
          []
      label: csapex::ExampleImageGenerator_0
    - state:
        persistent_params:
          []
        params:
          {}
      label: csapex::ExampleImageGenerator_0
      enabled: true
      muted: false
      minimized: false
      pos:
        - -603
        - 56
      logger_level: 1
      type: csapex::ExampleImageGenerator
      flipped: false
      exec_mode: 1
      color:
        - -1
        - -1
        - -1
      z: 15
      uuid: csapex::ExampleImageGenerator_9
    - minimized: false
      label: csapex::ExampleImageGenerator_0
      pos:
        - 33
        - 72
      uuid: csapex::ExampleImageGenerator_11
      exec_mode: 1
      type: csapex::ExampleImageGenerator
      enabled: true
      muted: false
      state:
        persistent_params:
          []
        params:
          {}
      logger_level: 1
      z: 12
      flipped: false
      color:
        - -1
        - -1
        - -1
    - minimized: false
      flipped: false
      pos:
        - -651
        - -263
      muted: false
      exec_mode: 1
      state:
        persistent_params:
          []
        params:
          {}
      enabled: true
      label: csapex::ExampleImageGenerator_0
      z: 4
      logger_level: 1
      color:
        - -1
        - -1
        - -1
      type: csapex::ExampleImageGenerator
      uuid: csapex::ExampleImageGenerator_0
    - logger_level: 1
      enabled: true
      type: csapex::ExampleImageGenerator
      uuid: csapex::ExampleImageGenerator_3
      label: csapex::ExampleImageGenerator_0
      exec_mode: 1
      z: 13
      pos:
        - 45
        - -17
      color:
        - -1
        - -1
        - -1
      minimized: false
      muted: false
      state:
        params:
          {}
        persistent_params:
          []
      flipped: false