#!/home/buck/ws/apex/devel/lib/csapex/csapex_main
nodes:
  - state:
      persistent_params:
        []
      params:
        h:
          type: value
          name: h
          int: 125
        text:
          type: value
          name: text
          string: "Tutorial Series: Adding New Tutorials\n\nThis tutorial shows the steps needed to add new tutorials in custom packages.\nFor an example see \"csapex_tutorials\", which contains this very tutorial configuration."
        w:
          type: value
          name: w
          int: 388
    type: csapex::Note
    uuid: csapex::Note_0
    max_frequency: 60
    label: prototype
    pos:
      - -159
      - -335
    color:
      - 0
      - 0
      - 0
    z: -2147483568
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
  - type: csapex::Note
    uuid: csapex::Note_3
    max_frequency: 60
    label: prototype
    pos:
      - -165
      - -192
    color:
      - -1
      - -1
      - -1
    z: -2147483550
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        h:
          type: value
          name: h
          int: 286
        text:
          type: value
          name: text
          string: "1. Create the directory structure:\n\n.../tutorials/README \n        The first line in this file is used as the title in the tutorials menu\n        For adding a new entry under \"Tutorials\", simply enter \"Tutorials\".\n\n.../tutorials/<number>_<name>\n        This folder contains a group of tutorials for similar topic <name1> \n        The tutorials are sorted by <number>\n\n.../tutorials/<number>_<name>/README\n         This file specifies the name of the tutorial in Line 1 and \n         an (optional) description in the following lines.\n\n.../tutorials/<number>_<name>/<number1>_<name_1>\n         This is an actual tutorial in for the topic <name>.\n         The folder must contain a \".apex\" configuration to load and a README as above."
        w:
          type: value
          name: w
          int: 564
      persistent_params:
        []
  - type: csapex::Note
    uuid: csapex::Note_4
    max_frequency: 60
    label: prototype
    pos:
      - -177
      - 111
    color:
      - -1
      - -1
      - -1
    z: -2147483540
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        h:
          type: value
          name: h
          int: 194
        text:
          type: value
          name: text
          string: "2. Export the tutorials:\n\nAdd a line similar to\n\n  <export>\n    <csapex plugin=\"${prefix}/plugins.xml\"\n            tutorials=\"${prefix}/tutorials\" />\n  </export>\n\nto your package.xml file."
        w:
          type: value
          name: w
          int: 399
      persistent_params:
        []
threads:
  groups: ~
  private_affinity:
    - true
    - true
    - true
    - true
  assignments:
    - uuid: csapex::Note_0
      id: 1
    - uuid: csapex::Note_3
      id: 1
    - uuid: csapex::Note_4
      id: 1
uuid_map:
  csapex::ExampleImageGenerator: 1
  csapex::Note: 5
  csapex::OutputDisplay: 1
settings:
  - string: access granted.
    name: access-test
    type: value
  - name: additional_args
    type: value
  - name: config
    type: value
    string: /home/buck/ws/apex/src/csapex_plugins/csapex_tutorial/tutorials/99_custom_tutorials/01_new_tutorial/01_tutorial.apex
  - type: value
    name: config_recovery
    bool: false
  - bool: false
    type: value
    name: debug
  - name: display-frequencies
    bool: true
    type: value
  - bool: false
    type: value
    name: display-graph-components
  - type: value
    name: display-minimap
    bool: false
  - type: value
    name: display-threads
    bool: false
  - type: value
    name: geometry
    string: AdnQywACAAAAAABBAAAALAAAB38AAARLAAAAQQAAACwAAAd/AAAESwAAAAACAAAAB4A=
  - name: grid
    bool: false
    type: value
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
    int: 12345
  - type: value
    name: schematics
    bool: false
  - bool: false
    type: value
    name: start-server
  - type: value
    name: test-observer
    string: initialized
  - type: value
    name: thread_grouping
    bool: true
  - bool: false
    type: value
    name: threadless
  - type: value
    name: uistate
    string: AAAA/wAAAAD9AAAAAwAAAAAAAAEkAAADzvwCAAAAAvsAAAAKAE4AbwBkAGUAcwAAAAAA/////wAAAM0A////+wAAABQASABlAGwAcABDAGUAbgB0AGUAcgEAAAA8AAADzgAAAOwA////AAAAAQAAAAAAAAAA/AIAAAAD+wAAAAoARABlAGIAdQBnAAAAAAD/////AAAAeQD////7AAAACABVAG4AZABvAAAAAAD/////AAAAzQD////7AAAAIABUAGgAcgBlAGEAZABNAGEAbgBhAGcAZQBtAGUAbgB0AAAAAAD/////AAABAAD///8AAAADAAAAAAAAAAD8AQAAAAL7AAAAEABUAGkAbQBlAGwAaQBuAGUAAAAAAP////8AAABtAP////sAAAASAFAAcgBvAGYAaQBsAGkAbgBnAAAAAAD/////AAAAjQD///8AAAYVAAADzgAAAAQAAAAEAAAACAAAAAj8AAAAAQAAAAIAAAACAAAACgBUAG8AbwBsAHMBAAAAAP////8AAAAAAAAAAAAAAA4ARgBpAGwAdABlAHIAcwEAAAL4/////wAAAAAAAAAA
connections:
  []
adapters:
  []