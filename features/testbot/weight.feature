@testbot
Feature: Weight tests

    Background:
        Given the profile "testbot"
        Given a grid size of 10 meters
        Given the extract extra arguments "--generate-edge-lookup"
        Given the query options
            | geometries | geojson |

    Scenario: Weight details
        Given the query options
            | annotations | true |

        Given the node map
            """
              s
              ·
            a---b---c
                    |
                    d
                    |··t
                    e
            """

        And the ways
            | nodes |
            | abc   |
            | cde   |

        When I route I should get
            | waypoints | route       | annotation                                                          |
            | s,t       | abc,cde,cde | 1.1:1.1:10.008843:0,2:2:20.017686:0,2:2:20.020734:0,1:1:10.010367:0 |

        When I route I should get
            | waypoints | route       | times      | weight_name | weights |
            | s,t       | abc,cde,cde | 3.1s,3s,0s | duration    | 3.1,3,0 |

    # FIXME include/engine/guidance/assemble_geometry.hpp:95
    @todo
    Scenario: Start and target on the same edge
        Given the query options
            | annotations | true |

        Given the node map
            """
            a-------b
              ·   ·
              s   t
            """

        And the ways
            | nodes |
            | ab    |

        When I route I should get
            | waypoints | route   | distances | weights | times | annotation         |
            | s,t       | abc,abc | 20m,0m    | 20,0    | 2s,0s | 29:2.9:20.017685:0 |
            | t,s       | abc,abc | 20m,0m    | 20,0    | 2s,0s | 29:2.9:20.017685:0 |

    # FIXME include/engine/guidance/assemble_geometry.hpp:95
    @todo
    Scenario: Start and target on adjacent edges
        Given the query options
            | annotations | true |

        Given the node map
            """
            a-------b-------c
              ·           ·
              s           t
            """

        And the ways
            | nodes |
            | ab    |

        When I route I should get
            | waypoints | route   | distances | weights | times   | annotation         |
            | s,t       | abc,abc | 30m,0m    | 31,0    | 3.1s,0s | 31:3.1:30.026527:0 |
            | t,s       | abc,abc | 30m,0m    | 31,0    | 3.1s,0s | 31:3.1:30.026527:0 |


    Scenario: Step weights -- way_function: fail if no weight or weight_per_meter property
        Given the profile file "testbot" extended with
        """
        properties.weight_name = 'steps'
        function way_function(way, result)
          result.forward_mode = mode.driving
          result.backward_mode = mode.driving
          result.forward_speed = 42
          result.backward_speed = 42
        end
        """
        And the node map
            """
            a---b
            """
        And the ways
            | nodes |
            | ab    |
        And the data has been saved to disk

        When I try to run "osrm-extract {osm_file} --profile {profile_file}"
        Then stderr should contain "There are no edges"
        And it should exit with an error

    Scenario: Step weights -- way_function: second way wins
        Given the profile file "testbot" extended with
        """
        properties.weight_name = 'steps'
        function way_function(way, result)
          result.forward_mode = mode.driving
          result.backward_mode = mode.driving
          result.duration = 42
          result.weight = 35
        end
        """

        Given the node map
            """
            a---b---c---d---e---f---g---h
            """

        And the ways
            | nodes    |
            | abcdef   |
            | abcdefgh |

        When I route I should get
            | waypoints | route | distance | weights | times  |
            | a,f       | ,     | 100m     | 25,0    | 30s,0s |
            | f,a       | ,     | 100m     | 25,0    | 30s,0s |
            | a,h       | ,     | 140m +-1 | 35,0    | 42s,0s |
            | h,a       | ,     | 140m +-1 | 35,0    | 42s,0s |

    Scenario: Step weights -- way_function: higher weight_per_meter is preferred
        Given the profile file "testbot" extended with
        """
        properties.weight_name = 'steps'
        function way_function(way, result)
          result.forward_mode = mode.driving
          result.backward_mode = mode.driving
          result.duration = 42
          result.forward_weight_per_meter = 1
          result.backward_weight_per_meter = 0.5
        end
        """

        Given the node map
            """
            a---b---c---d---e---f---g---h
            """

        And the ways
            | nodes    |
            | abcdefgh |
            | abcdef   |
            | fgh      |

        When I route I should get
            | waypoints | route | distance | weights | times  |
            | a,f       | ,     | 100m     | 99.9,0  | 30s,0s |
            | f,a       | ,     | 100m     | 199.8,0 | 30s,0s |
            | a,h       | ,     | 140m +-1 | 139.8,0 | 42s,0s |
            | h,a       | ,     | 140m +-1 | 279.6,0 | 42s,0s |
            | f,h       | ,     | 40m +-1  | 39.9,0  | 12s,0s |
            | h,f       | ,     | 40m +-1  | 79.8,0  | 12s,0s |

    Scenario: Step weights -- segment_function
        Given the profile file "testbot" extended with
        """
        properties.weight_name = 'steps'
        function way_function(way, result)
          result.forward_mode = mode.driving
          result.backward_mode = mode.driving
          result.weight = 42
          result.duration = 3
        end
        function segment_function (segment)
          segment.weight = 1
          segment.duration = 11
        end
        """

        Given the node map
            """
            a---b---c---d---e---f---g---h
            """

        And the ways
            | nodes    |
            | abcdefgh |
            | abcdef   |
            | fgh      |

        When I route I should get
            | waypoints | route | distance | weights | times  |
            | a,f       | ,     | 100m     | 5,0     | 55s,0s |
            | f,a       | ,     | 100m     | 5,0     | 55s,0s |
            | a,h       | ,     | 140m +-1 | 7,0     | 77s,0s |
            | h,a       | ,     | 140m +-1 | 7,0     | 77s,0s |
            | f,h       | ,     | 40m +-1  | 2,0     | 22s,0s |
            | h,f       | ,     | 40m +-1  | 2,0     | 22s,0s |
