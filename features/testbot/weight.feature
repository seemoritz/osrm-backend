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
            | waypoints | route       | annotation                                                            |
            | s,t       | abc,cde,cde | 11:1.1:10.008843:0,20:2:20.017686:0,20:2:20.020734:0,10:1:10.010367:0 |

        When I route I should get
            | waypoints | route       | times      | weight_name | weights |
            | s,t       | abc,cde,cde | 3.1s,3s,0s | duration    | 31,30,0 |

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
