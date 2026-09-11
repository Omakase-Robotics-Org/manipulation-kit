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


def test_chassis_sidewalls_and_wheel_covers_are_white():
    assert color_at('chassis_link',(-.20,0,.40))=='white'
    assert color_at('chassis_link',(0,.20,.10))=='white'
    assert color_at('chassis_link',(-.20,0,.45))=='navy'


def test_gallery_apron_has_rear_wheel_clearance():
    """The rear apron must not fill the caster opening seen in the gallery."""
    import pytest
    from manipulation_kit import assets
    path=assets.resolve('description/d1/meshes/body_hifi/chassis_link_refined.obj')
    if path is None:
        pytest.skip('Photo-refined private visual assets are not installed')
    vertices=[];apron=[];names=[];active=False
    for row in path.read_text().splitlines():
        fields=row.split()
        if not fields:continue
        if fields[0]=='v':vertices.append(tuple(map(float,fields[1:4])))
        elif fields[0]=='o':
            names.append(fields[1]);active=fields[1].startswith('Paint_white_LowerChassisCover')
        elif fields[0]=='f' and active:
            apron.extend(vertices[int(token.split('/')[0])-1] for token in fields[1:])
    rear=[p for p in apron if p[0]<-.26 and abs(p[1]-.0016)<.06]
    assert rear and min(p[2] for p in rear)>.09
    assert min(p[2] for p in apron)<.04
    assert sum(n.startswith('Paint_dark_CasterTread') for n in names)==4
    assert sum(n.startswith('Paint_dark_DriveTread') for n in names)==2
