#!/home/buck/ws/apex/devel/lib/csapex/csapex_main
settings:
  - type: value
    name: additional_args
  - name: config
    type: value
    string: 0_import_directory.apex
  - bool: false
    name: config_recovery
    type: value
  - type: value
    bool: false
    name: debug
  - type: value
    bool: false
    name: display-frequencies
  - type: value
    bool: false
    name: display-graph-components
  - name: display-minimap
    bool: false
    type: value
  - bool: false
    type: value
    name: display-threads
  - type: value
    name: geometry
    string: AdnQywACAAAAAAM/AAABUwAACs0AAAgwAAADPwAAAW8AAArNAAAIMAAAAAAAAAAADwA=
  - bool: false
    type: value
    name: grid
  - type: value
    name: grid-lock
    bool: false
  - bool: false
    type: value
    name: headless
  - list:
      []
    name: ignored_libraries
    type: string_list
  - type: value
    name: initially_paused
    bool: false
  - name: path_to_bin
    value: /home/buck/ws/apex/devel/lib/csapex/csapex_main
    type: path
  - name: port
    type: value
    int: 42123
  - type: value
    name: schematics
    bool: false
  - name: start-server
    bool: false
    type: value
  - type: value
    name: test-observer
    string: initialized
  - type: value
    name: thread_grouping
    bool: true
  - type: value
    name: threadless
    bool: false
  - string: AAAA/wAAAAD9AAAAAwAAAAAAAAAAAAAAAPwCAAAAAvsAAAAKAE4AbwBkAGUAcwAAAAAA/////wAAAMcA////+wAAABQASABlAGwAcABDAGUAbgB0AGUAcgAAAAAA/////wAAAOwA////AAAAAQAAAAAAAAAA/AIAAAAD+wAAAAoARABlAGIAdQBnAAAAAAD/////AAAAdgD////7AAAACABVAG4AZABvAAAAAAD/////AAAAxwD////7AAAAIABUAGgAcgBlAGEAZABNAGEAbgBhAGcAZQBtAGUAbgB0AAAAAAD/////AAAA/QD///8AAAADAAAAAAAAAAD8AQAAAAL7AAAAEABUAGkAbQBlAGwAaQBuAGUAAAAAAP////8AAABtAP////sAAAASAFAAcgBvAGYAaQBsAGkAbgBnAAAAAAD/////AAAAjQD///8AAAePAAAGcAAAAAQAAAAEAAAACAAAAAj8AAAAAQAAAAIAAAACAAAACgBUAG8AbwBsAHMBAAAAAP////8AAAAAAAAAAAAAAA4ARgBpAGwAdABlAHIAcwEAAAN6/////wAAAAAAAAAA
    type: value
    name: uistate
threads:
  groups:
    - id: 1
      name: default
      affinity:
        - true
        - true
        - true
        - true
        - true
        - true
        - true
        - true
        - true
        - true
        - true
        - true
  private_affinity:
    - true
    - true
    - true
    - true
    - true
    - true
    - true
    - true
    - true
    - true
    - true
    - true
  assignments:
    - uuid: ~
      id: 1
    - id: 1
      uuid: csapex::FileImporter_0
    - uuid: csapex::MarkerMessageDetector_0
      id: 1
    - uuid: csapex::CompareInteger_1
      id: 1
    - uuid: csapex::StringToInt_0
      id: 1
    - uuid: csapex::Sum_0
      id: 1
    - uuid: csapex::IntInput_1
      id: 1
    - uuid: csapex::BooleanTrigger_1
      id: 1
    - uuid: csapex::CompareInteger_0
      id: 1
    - uuid: csapex::CountMessages_0
      id: 1
    - uuid: csapex::IntInput_0
      id: 1
    - uuid: csapex::BinaryBoolean_1
      id: 1
    - uuid: csapex::Switch_0
      id: 1
    - uuid: csapex::TriggeredLatch_0
      id: 1
uuid_map:
  csapex::BinaryBoolean: 2
  csapex::BooleanTrigger: 4
  csapex::CompareInteger: 2
  csapex::CountMessages: 1
  csapex::Delay: 4
  csapex::FileImporter: 1
  csapex::IntInput: 2
  csapex::Latch: 2
  csapex::MarkerMessageDetector: 1
  csapex::SignalLight: 6
  csapex::StringToInt: 1
  csapex::Sum: 1
  csapex::Switch: 1
  csapex::TextDisplay: 6
  csapex::TiggerIfIntEqual: 1
  csapex::TriggeredLatch: 1
  csapex::boolean::Indicator: 1
nodes:
  - uuid: csapex::FileImporter_0
    pos:
      - -602
      - -206
    color:
      - -1
      - -1
      - -1
    label: csapex::FileImporter_0
    type: csapex::FileImporter
    max_frequency: 0
    z: 382
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        cache:
          type: value
          name: cache
          bool: false
        directory:
          type: path
          name: directory
          value: /home/buck/ws/apex/src/csapex_plugins/core_plugins/csapex_core_plugins/tests/res/input_sequence
        directory/current:
          type: range
          name: directory/current
          interactive: true
          int: 3
          min: 0
          max: 3
          step: 1
        directory/current_file:
          type: value
          name: directory/current_file
          string: /home/buck/ws/apex/src/csapex_plugins/core_plugins/csapex_core_plugins/tests/res/input_sequence/msg_2.apexm
        directory/filter:
          type: value
          name: directory/filter
          string: .*
        directory/latch:
          type: value
          name: directory/latch
          bool: false
        directory/loop:
          type: value
          name: directory/loop
          bool: false
        directory/play:
          type: value
          name: directory/play
          bool: false
        directory/quit on end:
          type: value
          name: directory/quit on end
          bool: false
        directory/show parameters:
          type: value
          name: directory/show parameters
          bool: false
        directory/sort_numerically:
          type: value
          name: directory/sort_numerically
          bool: true
        import directory:
          type: value
          name: import directory
          bool: true
        output_count:
          type: value
          name: output_count
          int: 1
        path:
          type: path
          name: path
          value: ../res/input_sequence/msg_0.apexm
        playback/resend:
          type: value
          name: playback/resend
          bool: false
        recursive import:
          type: value
          name: recursive import
          bool: false
        split_container_messages:
          type: value
          name: split_container_messages
          bool: false
      persistent_params:
        []
  - type: csapex::MarkerMessageDetector
    uuid: csapex::MarkerMessageDetector_0
    max_frequency: 0
    label: Trigger Final Check on Sequence End
    pos:
      - 635
      - 181
    color:
      - 181
      - 181
      - 181
    z: 365
    minimized: false
    muted: false
    enabled: true
    flipped: true
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        type:
          type: bitset
          name: type
          int: 6
          values:
            End Of Program: 4
            End Of Sequence: 2
            No Message: 1
      persistent_params:
        []
  - type: csapex::CompareInteger
    uuid: csapex::CompareInteger_1
    max_frequency: 0
    label: Message Content as Expected
    pos:
      - 282
      - 158
    color:
      - 255
      - 85
      - 0
    z: 378
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        operation:
          type: set
          name: operation
          txt: A == B
          int: 2
          values:
            -
              - A != B
              - 3
            -
              - A < B
              - 5
            -
              - A <= B
              - 4
            -
              - A == B
              - 2
            -
              - A > B
              - 0
            -
              - A >= B
              - 1
      persistent_params:
        []
  - type: csapex::StringToInt
    uuid: csapex::StringToInt_0
    max_frequency: 0
    label: csapex::StringToInt_0
    pos:
      - -60
      - 171
    color:
      - 255
      - 85
      - 0
    z: 374
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        {}
      persistent_params:
        []
  - max_frequency: 0
    uuid: csapex::Sum_0
    type: csapex::Sum
    label: File Content + Offset
    pos:
      - -53
      - 262
    color:
      - 255
      - 85
      - 0
    z: 372
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        input count:
          type: value
          name: input count
          int: 2
        input names:
          type: string_list
          name: input names
          list:
            - msg
            - 1
      persistent_params:
        []
  - type: csapex::IntInput
    uuid: csapex::IntInput_1
    max_frequency: 0
    label: Offset in File
    pos:
      - -363
      - 365
    color:
      - 255
      - 85
      - 0
    z: 381
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        value:
          type: value
          name: value
          int: 1
      persistent_params:
        []
  - type: csapex::BooleanTrigger
    uuid: csapex::BooleanTrigger_1
    max_frequency: 0
    label: Assertion Failed
    pos:
      - 295
      - 275
    color:
      - 255
      - 85
      - 0
    z: 362
    minimized: false
    muted: false
    enabled: true
    flipped: true
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        {}
      persistent_params:
        []
  - type: csapex::CompareInteger
    uuid: csapex::CompareInteger_0
    max_frequency: 0
    label: Number of Messages as Expected
    pos:
      - 291
      - -66
    color:
      - 255
      - 220
      - 41
    z: 266
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        operation:
          type: set
          name: operation
          txt: A == B
          int: 2
          values:
            -
              - A != B
              - 3
            -
              - A < B
              - 5
            -
              - A <= B
              - 4
            -
              - A == B
              - 2
            -
              - A > B
              - 0
            -
              - A >= B
              - 1
      persistent_params:
        []
  - uuid: csapex::CountMessages_0
    max_frequency: 0
    label: csapex::CountMessages_0
    pos:
      - -64
      - -180
    type: csapex::CountMessages
    color:
      - 255
      - 220
      - 41
    z: 380
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        reset:
          type: trigger
          name: reset
      persistent_params:
        []
  - type: csapex::IntInput
    uuid: csapex::IntInput_0
    max_frequency: 0
    label: Expected Messages
    pos:
      - -39
      - 29
    color:
      - 255
      - 220
      - 41
    z: 354
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        value:
          type: value
          name: value
          int: 3
      persistent_params:
        []
  - type: csapex::BinaryBoolean
    uuid: csapex::BinaryBoolean_1
    max_frequency: 0
    label: At the End And no Assert Fail
    pos:
      - 627
      - 37
    color:
      - 16
      - 110
      - 161
    z: 369
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        operation:
          type: set
          name: operation
          txt: A AND B
          int: 0
          values:
            -
              - A AND B
              - 0
            -
              - A NAND B
              - 2
            -
              - A NOR B
              - 3
            -
              - A OR B
              - 1
            -
              - A XOR B
              - 4
      persistent_params:
        []
  - type: csapex::Switch
    uuid: csapex::Switch_0
    max_frequency: 0
    label: Check if Assertions Pass
    pos:
      - 420
      - 417
    color:
      - 181
      - 181
      - 181
    z: 352
    minimized: false
    muted: false
    enabled: true
    flipped: true
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        predicate:
          type: value
          name: predicate
          interactive: true
          bool: false
      persistent_params:
        []
  - type: csapex::TriggeredLatch
    uuid: csapex::TriggeredLatch_0
    max_frequency: 0
    label: Cache Last Assertion
    pos:
      - 714
      - 419
    color:
      - 181
      - 181
      - 181
    z: 348
    minimized: false
    muted: false
    enabled: true
    flipped: true
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        ignore_marker_tokens:
          type: value
          name: ignore_marker_tokens
          bool: true
      persistent_params:
        []
adapters:
  []
connections:
  - uuid: event_activation
    targets:
      - csapex::FileImporter_0:|:slot_2
      - csapex::CountMessages_0:|:slot_2
    types:
      - default
      - default
  - uuid: csapex::MarkerMessageDetector_0:|:event_3
    targets:
      - csapex::TriggeredLatch_0:|:slot_4
    types:
      - default
  - uuid: csapex::CountMessages_0:|:event_3
    targets:
      - csapex::FileImporter_0:|:slot_3
    types:
      - default
  - uuid: csapex::TriggeredLatch_0:|:out_0
    targets:
      - csapex::Switch_0:|:in_0
      - csapex::Switch_0:|:in_predicate
    types:
      - default
      - default
  - uuid: csapex::IntInput_1:|:out_0
    targets:
      - csapex::Sum_0:|:in_1
    types:
      - default
  - uuid: csapex::FileImporter_0:|:out_0
    targets:
      - csapex::StringToInt_0:|:in_0
      - csapex::CountMessages_0:|:in_0
    types:
      - default
      - default
  - uuid: csapex::IntInput_0:|:out_0
    targets:
      - csapex::CompareInteger_0:|:in_1
    types:
      - default
  - uuid: csapex::StringToInt_0:|:out_0
    targets:
      - csapex::CompareInteger_1:|:in_0
    types:
      - default
  - uuid: csapex::Sum_0:|:out_0
    targets:
      - csapex::CompareInteger_1:|:in_1
    types:
      - default
  - uuid: csapex::Switch_0:|:out_0
    targets:
      - slot_exit
    types:
      - default
  - uuid: csapex::CountMessages_0:|:out_0
    targets:
      - csapex::CompareInteger_0:|:in_0
    types:
      - default
  - uuid: csapex::CompareInteger_1:|:out_0
    targets:
      - csapex::BooleanTrigger_1:|:in_0
      - csapex::BinaryBoolean_1:|:in_1
    types:
      - default
      - default
  - uuid: csapex::CompareInteger_0:|:out_0
    targets:
      - csapex::BinaryBoolean_1:|:in_0
    types:
      - default
  - uuid: csapex::BinaryBoolean_1:|:out_0
    targets:
      - csapex::MarkerMessageDetector_0:|:in_0
      - csapex::TriggeredLatch_0:|:slot_2
    types:
      - default
      - default
  - uuid: csapex::FileImporter_0:|:out_directory/current
    targets:
      - csapex::Sum_0:|:in_0
    types:
      - default
  - uuid: csapex::BooleanTrigger_1:|:event_4
    targets:
      - slot_abort
    types:
      - default
  - uuid: csapex::Switch_0:|:out_1
    targets:
      - slot_abort
    types:
      - default