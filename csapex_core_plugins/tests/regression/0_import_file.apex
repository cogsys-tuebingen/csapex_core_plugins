#!/home/buck/ws/apex/devel/lib/csapex/csapex_main
threads:
  assignments:
    - uuid: ~
      id: 1
    - id: 1
      uuid: csapex::FileImporter_0
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
settings:
  - name: additional_args
    type: value
  - name: config
    type: value
    string: /home/buck/ws/apex/src/csapex_plugins/core_plugins/csapex_core_plugins/tests/regression/import_file_and_exit.apex
  - type: value
    name: config_recovery
    bool: false
  - type: value
    name: debug
    bool: false
  - type: value
    name: display-frequencies
    bool: false
  - name: display-graph-components
    bool: false
    type: value
  - type: value
    name: display-minimap
    bool: false
  - type: value
    name: display-threads
    bool: false
  - type: value
    name: geometry
    string: AdnQywACAAAAAAR7AAACNQAACP8AAAXwAAAEewAAAlEAAAj/AAAF8AAAAAAAAAAADwA=
  - type: value
    name: grid
    bool: false
  - type: value
    name: grid-lock
    bool: false
  - type: value
    name: headless
    bool: false
  - type: string_list
    name: ignored_libraries
    list:
      []
  - type: value
    name: initially_paused
    bool: false
  - type: path
    name: path_to_bin
    value: /home/buck/ws/apex/devel/lib/csapex/csapex_main
  - type: value
    name: port
    int: 42123
  - type: value
    name: schematics
    bool: false
  - type: value
    name: start-server
    bool: false
  - type: value
    name: test-observer
    string: initialized
  - type: value
    name: thread_grouping
    bool: true
  - type: value
    name: threadless
    bool: false
  - type: value
    name: uistate
    string: AAAA/wAAAAD9AAAAAwAAAAAAAAAAAAAAAPwCAAAAAvsAAAAKAE4AbwBkAGUAcwAAAAAA/////wAAAMcA////+wAAABQASABlAGwAcABDAGUAbgB0AGUAcgAAAAAA/////wAAAOwA////AAAAAQAAAAAAAAAA/AIAAAAD+wAAAAoARABlAGIAdQBnAAAAAAD/////AAAAdgD////7AAAACABVAG4AZABvAAAAAAD/////AAAAxwD////7AAAAIABUAGgAcgBlAGEAZABNAGEAbgBhAGcAZQBtAGUAbgB0AAAAAAD/////AAAA/QD///8AAAADAAAAAAAAAAD8AQAAAAL7AAAAEABUAGkAbQBlAGwAaQBuAGUAAAAAAP////8AAABtAP////sAAAASAFAAcgBvAGYAaQBsAGkAbgBnAAAAAAD/////AAAAjQD///8AAASFAAADTgAAAAQAAAAEAAAACAAAAAj8AAAAAQAAAAIAAAACAAAACgBUAG8AbwBsAHMBAAAAAP////8AAAAAAAAAAAAAAA4ARgBpAGwAdABlAHIAcwEAAAL4/////wAAAAAAAAAA
uuid_map:
  csapex::FileImporter: 1
nodes:
  - type: csapex::FileImporter
    uuid: csapex::FileImporter_0
    max_frequency: 60
    label: csapex::FileImporter_0
    pos:
      - -285
      - -135
    color:
      - -1
      - -1
      - -1
    z: 6
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
          value: ""
        directory/current:
          type: range
          name: directory/current
          int: 0
          min: 0
          max: 0
          step: 1
        directory/current_file:
          type: value
          name: directory/current_file
          string: ""
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
          bool: true
        directory/play:
          type: value
          name: directory/play
          bool: true
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
          bool: false
        import directory:
          type: value
          name: import directory
          bool: false
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
adapters:
  []
connections:
  - targets:
      - slot_exit
    types:
      - default
    uuid: csapex::FileImporter_0:|:out_0
