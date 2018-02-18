#!/home/buck/ws/apex/devel/lib/csapex/csapex_main
threads:
  assignments:
    - uuid: ~
      id: 1
    - id: 1
      uuid: csapex::Delay_0
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
  - type: value
    name: additional_args
  - name: config
    string: succeed.apex
    type: value
  - type: value
    name: config_recovery
    bool: false
  - bool: false
    type: value
    name: debug
  - type: value
    name: display-frequencies
    bool: false
  - type: value
    name: display-graph-components
    bool: false
  - type: value
    name: display-minimap
    bool: false
  - type: value
    name: display-threads
    bool: false
  - type: value
    name: geometry
    string: AdnQywACAAAAAABBAAADnAAABMUAAAdgAAAAQQAAA7gAAATFAAAHYAAAAAAAAAAADwA=
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
    name: thread_grouping
    bool: true
  - type: value
    name: threadless
    bool: false
  - type: value
    name: uistate
    string: AAAA/wAAAAD9AAAAAwAAAAAAAAAAAAAAAPwCAAAAAvsAAAAKAE4AbwBkAGUAcwAAAAAA/////wAAAMcA////+wAAABQASABlAGwAcABDAGUAbgB0AGUAcgAAAAAA/////wAAAOwA////AAAAAQAAAAAAAAAA/AIAAAAD+wAAAAoARABlAGIAdQBnAAAAAAD/////AAAAdgD////7AAAACABVAG4AZABvAAAAAAD/////AAAAxwD////7AAAAIABUAGgAcgBlAGEAZABNAGEAbgBhAGcAZQBtAGUAbgB0AAAAAAD/////AAAA/QD///8AAAADAAAAAAAAAAD8AQAAAAL7AAAAEABUAGkAbQBlAGwAaQBuAGUAAAAAAP////8AAABtAP////sAAAASAFAAcgBvAGYAaQBsAGkAbgBnAAAAAAD/////AAAAjQD///8AAASFAAADVwAAAAQAAAAEAAAACAAAAAj8AAAAAQAAAAIAAAACAAAACgBUAG8AbwBsAHMBAAAAAP////8AAAAAAAAAAAAAAA4ARgBpAGwAdABlAHIAcwEAAAN6/////wAAAAAAAAAA
uuid_map:
  csapex::Delay: 1
nodes:
  - max_frequency: 60
    label: csapex::Delay_0
    pos:
      - -64
      - -130
    uuid: csapex::Delay_0
    type: csapex::Delay
    color:
      - -1
      - -1
      - -1
    z: 4
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        blocking:
          type: value
          name: blocking
          bool: false
        delay:
          type: range
          name: delay
          double: 0.5
          min: 0
          max: 10
          step: 0.01
        delay progress:
          type: progress
          name: delay progress
      persistent_params:
        []
adapters:
  []
connections:
  - targets:
      - csapex::Delay_0:|:in_0
    types:
      - default
    uuid: event_activation
  - uuid: csapex::Delay_0:|:out_0
    targets:
      - slot_exit
    types:
      - default