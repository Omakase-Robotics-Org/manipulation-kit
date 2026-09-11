"""Rear controls must remain distinguishable from the painted panel."""
from manipulation_kit.description.d1.tools.build_visual_colors import regions


def color_at(host, point):
    for color, planes in regions(host):
        if all(sum(a*b for a,b in zip(plane[:3],point))+plane[3]>=-1e-10 for plane in planes):
            return color
    return 'white'


def test_rear_emergency_stop_and_panel_from_photos():
    assert color_at('torso_column',(-.105,.0016,.550))=='red'
    assert color_at('torso_column',(-.10,.0016,.510))=='navy'
    assert color_at('torso_column',(-.10,.080,.510))=='white'
    assert color_at('torso_column',(-.10,.0016,.460))=='white'


def test_lift_cover_is_white_not_a_continuation_of_blue_waist():
    assert color_at('torso_column',(-.04,0,-.10))=='white'
    assert color_at('chassis_link',(-.043,0,.50))=='white'
    assert color_at('chassis_link',(-.20,0,.44))=='navy'
