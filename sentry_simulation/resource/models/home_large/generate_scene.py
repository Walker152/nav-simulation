#!/usr/bin/env python3
"""Reproduce the original single-floor home, maps and inspection plan locally.

Requires the package's numpy/yaml dependencies. Geometry is authored in metres;
all furniture visuals have matching primitive collisions. No downloaded assets.
"""
from copy import deepcopy
from html import escape
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
COLORS = {
    'wall': '#d5d9d6', 'frame': '#796553', 'floor': '#c7b99f',
    'wood': '#956c45', 'lightwood': '#c6a87a', 'white': '#e8e9e2',
    'dark': '#303b46', 'metal': '#616f76', 'sofa': '#537d80',
    'bed': '#8ba6bf', 'pink': '#d78d9c', 'red': '#cc6555',
    'yellow': '#e4b44b', 'blue': '#488bb6', 'green': '#67875b',
    'leaf': '#648c60', 'pot': '#af7356', 'rug': '#ae9680', 'skin': '#c99672',
}


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def numbers(values):
    return ' '.join(f'{x:.9g}' for x in values)


def write_xml(root, path):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def rgba(color):
    value = COLORS.get(color, color).lstrip('#')
    return numbers([int(value[i:i+2], 16)/255 for i in (0, 2, 4)] + [1])


class Scene:
    def __init__(self):
        self.root = ET.Element('sdf', version='1.9')
        model = element(self.root, 'model', name='home_large')
        element(model, 'static', 'true')
        self.link = element(model, 'link', name='house')
        self.items = []
        self.doors = []
        config = yaml.safe_load((PACKAGE / 'config/swerve_odin.yaml').read_text())
        odin = ET.parse(PACKAGE / 'resource/models/odin1_lite/model.sdf')
        sensor_x = float(odin.findtext('.//collision/geometry/box/size').split()[0])
        self.robot_width = config['geometry']['width'] + sensor_x

    def shape(self, name, xyz, dims, color='wood', yaw=0, kind='box', link=None, collision=True):
        target = self.link if link is None else link
        geometry = ET.Element('geometry')
        shape = element(geometry, kind)
        if kind == 'box':
            element(shape, 'size', numbers(dims))
        elif kind == 'cylinder':
            element(shape, 'radius', dims[0])
            element(shape, 'length', dims[1])
        elif kind == 'sphere':
            element(shape, 'radius', dims[0])
        else:
            raise ValueError(kind)
        for tag in (('collision', 'visual') if collision else ('visual',)):
            node = element(target, tag, name=name if tag == 'collision' else name+'_visual')
            element(node, 'pose', numbers((*xyz, 0, 0, yaw)))
            node.append(deepcopy(geometry))
            if tag == 'visual':
                material = element(node, 'material')
                element(material, 'ambient', rgba(color))
                element(material, 'diffuse', rgba(color))
            else:
                friction = element(element(element(node, 'surface'), 'friction'), 'ode')
                element(friction, 'mu', 1)
                element(friction, 'mu2', 1)
        if link is None:
            self.items.append(dict(name=name, xyz=xyz, dims=dims, color=COLORS.get(color, color),
                                   yaw=yaw, kind=kind, collision=collision))

    def box(self, name, xyz, dims, color='wood', yaw=0):
        self.shape(name, xyz, dims, color, yaw)

    def cylinder(self, name, x, y, height, radius, color='metal'):
        self.shape(name, (x, y, height/2), (radius, height), color, kind='cylinder')

    def wall(self, name, x, y, sx, sy):
        self.box(name, (x, y, 1.4), (sx, sy, 2.8), 'wall')

    def doorway(self, name, x0, x1, y, side_clearance):
        x = (x0+x1)/2
        width = self.robot_width + 2 * side_clearance
        # Door width is the distance BETWEEN the inner jamb collision faces.
        for side, a, b in (('left', x0, x-width/2-.07), ('right', x+width/2+.07, x1)):
            self.wall(name+'_wall_'+side, (a+b)/2, y, b-a, .18)
            sign = -1 if side == 'left' else 1
            self.box(name+'_'+side, (x+sign*(width/2+.035), y, 1.05), (.07, .24, 2.1), 'frame')
        self.box(name+'_lintel', (x, y, 2.5), (width+.14, .18, .6), 'wall')
        self.box(name+'_top', (x, y, 2.15), (width+.14, .24, .10), 'frame')
        self.doors.append(dict(name=name, center=[x, y], clear_width=width,
                               side_clearance=side_clearance, clear_height=2.1))

    def table(self, name, x, y, sx=2, sy=1.0, height=.78, yaw=0):
        self.box(name+'_top', (x, y, height-.035), (sx, sy, .07), 'lightwood', yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        for i, dx in enumerate((-sx/2+.10, sx/2-.10)):
            for j, dy in enumerate((-sy/2+.10, sy/2-.10)):
                self.box(f'{name}_leg_{i}{j}', (x+c*dx-s*dy, y+s*dx+c*dy, (height-.07)/2),
                         (.065, .065, height-.07), 'dark', yaw)

    def chair(self, name, x, y, yaw=0, height=.46):
        c, s = math.cos(yaw), math.sin(yaw)
        def part(suffix, dx, dy, z, dims, color):
            self.box(name+suffix, (x+c*dx-s*dy, y+s*dx+c*dy, z), dims, color, yaw)
        part('_seat', 0, 0, height, (.48, .48, .07), 'sofa')
        part('_back', 0, .215, height+.22, (.48, .065, .44), 'wood')
        for i, dx in enumerate((-.18, .18)):
            for j, dy in enumerate((-.18, .18)):
                part(f'_leg_{i}{j}', dx, dy, height/2, (.045, .045, height), 'wood')

    def bed(self, name, x, y, single=False, color='bed'):
        width = 1.2 if single else 2.0
        self.box(name+'_frame', (x, y, .23), (width, 2.2, .46), 'wood')
        self.box(name+'_mattress', (x, y, .54), (width-.05, 2.15, .16), 'white')
        self.box(name+'_duvet', (x, y-.24, .64), (width-.02, 1.6, .10), color)
        self.box(name+'_head', (x, y+1.05, .65), (width+.10, .12, 1.3), 'wood')
        for i, dx in enumerate((0,) if single else (-.5, .5)):
            self.box(name+f'_pillow_{i}', (x+dx, y+.76, .66), (.7, .4, .12), 'white')
        for i, dx in enumerate((-width/2-.42, width/2+.42)):
            self.box(name+f'_nightstand_{i}', (x+dx, y+.70, .30), (.55, .6, .6), 'lightwood')

    def sofa(self, name, x, y, length=3.2):
        self.box(name+'_base', (x, y, .28), (length, 1, .56), 'sofa')
        self.box(name+'_back', (x, y+.43, .74), (length, .16, .8), 'sofa')
        for i, dx in enumerate((-length/2+.1, length/2-.1)):
            self.box(name+f'_arm{i}', (x+dx, y, .60), (.20, 1, .35), 'sofa')
        for i, dx in enumerate((-.85, 0, .85)):
            self.box(name+f'_cushion{i}', (x+dx, y-.04, .60), (.78, .72, .13), 'bed')

    def plant(self, name, x, y, height=1.1):
        self.cylinder(name+'_pot', x, y, .36, .23, 'pot')
        self.cylinder(name+'_stem', x, y, height, .035, 'wood')
        for i, (dx, dy, z, r) in enumerate(((0, 0, height, .29), (.16, .06, height-.22, .24), (-.12, -.1, height-.3, .22))):
            self.shape(name+f'_leaf{i}', (x+dx, y+dy, z), (r,), 'leaf', kind='sphere')

    def slipper(self, name, x, y, yaw=0, color='pink'):
        # Rounded toe, a low sole and a raised strap; not a tall bounding box.
        self.box(name+'_sole', (x, y, .015), (.24, .11, .03), color, yaw)
        self.shape(name+'_toe', (x+.105*math.cos(yaw), y+.105*math.sin(yaw), .017),
                   (.055, .034), color, kind='cylinder')
        self.box(name+'_strap', (x+.025*math.cos(yaw), y+.025*math.sin(yaw), .052),
                 (.08, .11, .042), 'white', yaw)


def architecture(scene):
    scene.box('floor', (24, 16, -.1), (48, 32, .2), 'floor')
    for name, x, y, sx, sy in (('south', 24, 0, 48, .2), ('north', 24, 32, 48, .2),
                              ('west', 0, 16, .2, 32), ('east', 48, 16, .2, 32)):
        scene.wall('outer_'+name, x, y, sx, sy)
    # Per-side clearance in metres; keep narrow challenges among wider everyday doors.
    south = [('foyer', '玄关', .30), ('living', '客厅', .30), ('dining', '餐厅', .25),
             ('kitchen', '厨房', .25), ('laundry', '洗衣房', .15), ('service_balcony', '生活阳台', .20)]
    north = [('master', '主卧', .30), ('wardrobe', '衣帽间', .15), ('master_bath', '主卫', .10),
             ('child_bedroom', '儿童卧室', .20), ('second_bedroom', '次卧', .25), ('guest', '客卧', .20)]
    rooms = []
    for row, names, y0, y1, door_y in (('s', south, 0, 8.5, 8.5), ('n', north, 23.5, 32, 23.5)):
        for i, (key, label, clearance) in enumerate(names):
            x0, x1 = i*8, (i+1)*8
            if i:
                scene.wall(f'{row}_divide{i}', x0, (y0+y1)/2, .18, y1-y0)
            scene.doorway(key+'_door', x0, x1, door_y, clearance)
            goal_y = y1-1.4 if row == 's' else y0+1.4
            rooms.append(dict(name=key, label=label, bounds=[x0,y0,x1,y1], goal=[(x0+x1)/2,goal_y]))
    for wing, x0, names in (('w', 3, [('study','书房',.20), ('playroom','玩具房',.30), ('gym','健身房',.25)]),
                            ('e', 25.5, [('storage','储藏室',.10), ('bathroom','公卫',.15), ('leisure_balcony','休闲阳台',.30)])):
        for i in range(4):
            scene.wall(f'{wing}_divide{i}', x0+i*6.5, 16, .18, 9)
        for i, (key, label, clearance) in enumerate(names):
            a, b = x0+i*6.5, x0+(i+1)*6.5
            scene.doorway(key+'_south_door', a, b, 11.5, clearance)
            scene.doorway(key+'_north_door', a, b, 20.5, clearance)
            rooms.append(dict(name=key, label=label, bounds=[a,11.5,b,20.5], goal=[(a+b)/2,12.9]))
    # Visual-only floor finishes share the one physical ground plane.
    for i, room in enumerate(rooms):
        x0,y0,x1,y1 = room['bounds']
        color = ['#d7c4a6','#c9c7bb','#b9c7c6','#d3c0b1'][i%4]
        scene.shape(room['name']+'_finish', ((x0+x1)/2,(y0+y1)/2,.0005),
                    (x1-x0-.2,y1-y0-.2,.001), color, collision=False)
    return rooms


def furnish(s, rooms):
    for room in rooms:
        name = room['name']
        x0,y0,x1,y1 = room['bounds']
        cx, cy = (x0+x1)/2, (y0+y1)/2
        # Keep a clear centre approach to every doorway. Furniture is placed
        # asymmetrically, leaving room for long-body turns on either side.
        s.plant(name+'_plant', x1-.7, y0+.85, .9)
        if name in ('master','second_bedroom','guest','child_bedroom'):
            s.bed(name+'_bed', x0+2.2, y0+5.2, name=='child_bedroom', 'pink' if name=='child_bedroom' else 'bed')
            s.box(name+'_wardrobe', (x1-.55,y0+5.2,1.1), (.75,3,2.2), 'white')
            s.table(name+'_desk', x1-1.7,y1-.65,2,.7)
            s.chair(name+'_chair', x1-1.6,y1-1.55,.24)
            s.slipper(name+'_slipper_a', x0+3.65,y0+4.4,.6)
            s.slipper(name+'_slipper_b', x0+3.9,y0+4.1,-.4)
        elif name == 'foyer':
            s.box('shoe_cabinet', (1,3.8,.6), (.6,3.2,1.2), 'white')
            s.table('entry_bench', 6,3,1.4,.6,.45)
            s.cylinder('coat_stand', 6.4,5.4,1.8,.045)
            s.cylinder('coat_stand_base', 6.4,5.4,.05,.28)
            for i, (x,y,a) in enumerate(((2.2,2,.5),(2.6,2.2,-.4),(5.8,4.2,1.3),(6.2,4.4,-.3))):
                s.slipper('entry_slipper'+str(i),x,y,a)
        elif name == 'living':
            s.sofa('living_sofa',cx,2.2)
            s.sofa('living_small_sofa',x0+2,5,2.2)
            s.table('coffee_table',cx,4,1.8,.85,.42,.12)
            s.box('tv_console',(x1-.55,4.7,.3),(.65,3,.6),'wood')
            s.box('television',(x1-.5,4.7,1.15),(.12,1.8,.9),'dark')
            s.box('living_cart',(x0+1.1,6.4,.25),(.65,.55,.5),'yellow',-.25)
        elif name == 'dining':
            s.table('dining_table',cx,3.7,3.2,1.2)
            for i, dx in enumerate((-1.1,0,1.1)):
                s.chair('dining_chair_s'+str(i),cx+dx,2.6,math.pi+.12*i)
                s.chair('dining_chair_n'+str(i),cx+dx,4.8,.10*i)
            s.box('dining_sideboard',(x0+.6,3.8,.5),(.7,3,1),'white')
            s.table('high_table',x1-1.3,6,1.3,.65,1.05,.3)
        elif name == 'kitchen':
            s.box('kitchen_counter_s',(cx, .6,.46),(6.4,.9,.92),'white')
            s.box('kitchen_counter_e',(x1-.6,3.2,.46),(.9,4.2,.92),'white')
            s.box('fridge',(x0+.8,1.1,1.05),(1.0,1.0,2.1),'metal')
            s.box('oven',(cx-1.5,.12,.5),(.75,.06,.6),'dark')
            s.box('sink',(cx+1,.6,.94),(.85,.65,.04),'metal')
            for i in (-1,1):
                s.shape('stove'+str(i),(cx-.2+i*.19,.6,.94),(.14,.025),'dark',kind='cylinder')
            s.box('kitchen_island',(cx,3.9,.48),(2.6,1.1,.96),'wood')
            s.table('breakfast_bar',cx,5.8,2.7,.7,1.05)
            s.chair('bar_stool_a',cx-.9,6.6,.2,.73)
            s.chair('bar_stool_b',cx+.9,6.5,-.15,.73)
            s.cylinder('kitchen_bin',x1-1.2,6,.6,.23,'dark')
        elif name == 'laundry':
            for i in range(2):
                s.box('washer'+str(i),(x0+1.2+i*1.2,1,.45),(.85,.85,.9),'white')
                s.box('washer_window'+str(i),(x0+1.2+i*1.2,1.435,.45),(.55,.02,.5),'dark')
            s.box('laundry_cabinet',(x1-.6,3.8,1),(.7,3.5,2),'white')
            s.table('ironing_board',cx,4,1.6,.5,.9,.35)
            s.cylinder('laundry_basket',x0+1.3,5,.5,.32,'rug')
        elif name in ('master_bath','bathroom'):
            s.box(name+'_bath',(x0+1.45,y1-1.25,.3),(2.1,1.3,.6),'white')
            s.box(name+'_vanity',(x1-.6,cy,.42),(.8,2.0,.84),'wood')
            s.box(name+'_basin',(x1-.6,cy,.88),(.85,1.8,.08),'white')
            s.cylinder(name+'_toilet',x0+1.1,cy,.42,.25,'white')
            s.box(name+'_cistern',(x0+.85,cy,.65),(.25,.6,.65),'white')
            s.box(name+'_shower_screen',(x0+2.4,y1-2.4,1.0),(2.8,.04,2),'blue')
        elif name == 'wardrobe':
            for side in (-1,1):
                s.box('wardrobe_bank'+str(side),(cx+side*2.9,cy,1.15),(.65,5.8,2.3),'lightwood')
                for j in range(5):
                    s.box(f'wardrobe_handle{side}_{j}',(cx+side*2.54,y0+2+j,.95),(.045,.035,.28),'dark')
            s.table('dressing_bench',cx,cy,1.6,.6,.45)
            s.box('suitcase',(x0+1.2,y1-1,.4),(.5,.7,.8),'blue',.2)
        elif name == 'study':
            s.table('study_desk',x0+1.6,cy,2,.85)
            s.chair('study_chair',x0+1.6,cy-1,.2)
            s.box('study_monitor',(x0+1.6,cy+.2,1.08),(.75,.06,.48),'dark')
            for j in range(2):
                s.box('bookcase'+str(j),(x1-.45,y0+2.5+j*3,1.1),(.55,2.2,2.2),'wood')
                for k in range(4):
                    s.box(f'books_{j}_{k}',(x1-.77,y0+2+j*3,.35+k*.45),(.08,1.1,.30),'blue' if k%2 else 'red')
        elif name == 'playroom':
            s.box('toy_shelf',(x0+.5,cy,.45),(.6,3,.9),'yellow')
            s.table('child_table',x1-1.6,y1-2,1.6,.8,.52)
            s.chair('child_chair',x1-1.4,y1-3,.2,.30)
            # Open tent: two sloping roof panels are represented with yaw-free
            # stacked narrow tiers, preserving a non-box silhouette and void.
            for j in range(6):
                for sign in (-1,1):
                    s.box(f'tent_{j}_{sign}',(x1-1.2+sign*(.70-.10*j),cy,.12+j*.18),(.16,1.2,.22),'pink')
            for i, (dx,dy) in enumerate(((1.5,2),(2,2.5),(4.6,2.2),(4.8,3.6),(1.8,5.8),(2.1,6.1))):
                s.box('toy_block'+str(i),(x0+dx,y0+dy,.045+i%3*.015),(.14,.10,.09+i%3*.03),('red','yellow','blue')[i%3],i*.63)
            for i, (dx,dy) in enumerate(((1.9,3.4),(4.5,5.0))):
                s.shape('toy_ball'+str(i),(x0+dx,y0+dy,.12),(.12,), 'blue' if i else 'yellow',kind='sphere')
            s.box('toy_car',(x0+1.8,y0+4.5,.11),(.38,.20,.16),'red',.6)
            s.box('toy_car_cabin',(x0+1.8,y0+4.5,.22),(.18,.17,.10),'blue',.6)
        elif name == 'gym':
            s.box('treadmill_base',(x0+1.3,cy,.13),(1,2,.26),'dark')
            for dx in (-.42,.42):
                s.box('treadmill_rail'+str(dx),(x0+1.3+dx,cy+.65,.65),(.07,.07,1.2),'metal')
            s.box('gym_mat',(x1-1.4,cy,.012),(.8,2,.024),'blue',.15)
            s.shape('exercise_ball',(x1-1.2,y1-1.2,.32),(.32,),'pink',kind='sphere')
        elif name == 'storage':
            for i in range(3):
                s.box('storage_shelf'+str(i),(x0+.55,y0+1.8+i*2.4,1),(.7,1.8,2),'metal')
            for i, (dx,dy,a) in enumerate(((4.8,2,.3),(4.6,3.1,-.2),(5.1,5,.5),(4.3,6.2,-.3))):
                s.box('carton'+str(i),(x0+dx,y0+dy,.24+i%2*.1),(.7,.6,.48+i%2*.2),'lightwood',a)
            s.box('folded_stroller',(x1-.8,y1-1,.6),(.5,.7,1.2),'dark',.2)
        elif name in ('service_balcony','leisure_balcony'):
            for i in range(3):
                s.plant(name+'_extra_plant'+str(i),x0+.7,y0+2.2+i*1.8,1.0+i*.12)
            if name == 'service_balcony':
                for i, dx in enumerate((-1.1,1.1)):
                    s.cylinder('drying_rack_post'+str(i),cx+dx,cy,1.6,.025)
                    s.box('drying_rack_foot'+str(i),(cx+dx,cy,.03),(.10,1,.06),'metal')
                s.box('drying_rack_top',(cx,cy,1.62),(2.25,.06,.06),'metal')
                for j in range(4):
                    s.box('hanging_towel'+str(j),(cx-.8+j*.5,cy,.99),(.30,.035,.8),'blue' if j%2 else 'white')
            else:
                s.table('balcony_high_table',cx+1,cy,1.3,.75,1.05)
                s.chair('balcony_chair',cx+1,cy-1,.3,.7)
                s.box('balcony_low_planter',(x1-.5,y1-1.6,.25),(.6,1.8,.5),'pot')
    # Deliberately displaced clutter on hall shoulders leaves the central ring clear.
    for i, (x,y,a) in enumerate(((8.2,8.95,.3),(8.55,9.05,-.5),(34.8,22.95,.7),(35.2,22.85,-.2))):
        s.slipper('hall_slipper'+str(i),x,y,a,'blue')
    s.box('hall_parcel',(30.5,8.95,.16),(.5,.4,.32),'lightwood',.35)


def people(s, world):
    configs = [
        ('adult_hall',1.75,'blue',[(0,18,10,0),(8,24,10,0),(10,24,10,math.pi),(18,18,10,math.pi),(20,18,10,0)]),
        ('adult_north',1.68,'green',[(0,29,21.8,0),(7,34,21.8,0),(9,34,21.8,math.pi),(16,29,21.8,math.pi),(18,29,21.8,0)]),
        ('child',1.05,'yellow',[(0,12.75,10.2,math.pi/2),(9,12.75,14,math.pi/2),(11,12.75,14,-math.pi/2),(20,12.75,10.2,-math.pi/2),(22,12.75,10.2,math.pi/2)]),
        ('standing',1.72,'red',[(0,42.6,17.6,0)]),
    ]
    metadata=[]
    for name,height,color,points in configs:
        model=element(world,'model',name='person_'+name)
        element(model,'static','true')
        _,x,y,yaw=points[0]
        element(model,'pose',numbers((x,y,0,0,0,yaw)))
        link=element(model,'link',name='body')
        q=height/1.75
        for side in (-1,1):
            s.shape('leg'+str(side),(0,side*.11*q,.39*q),(.065*q,.78*q),'dark',kind='cylinder',link=link)
            s.shape('foot'+str(side),(.06*q,side*.11*q,.055*q),(.27*q,.14*q,.11*q),'dark',link=link)
            s.shape('arm'+str(side),(0,side*.245*q,1.10*q),(.055*q,.60*q),color,kind='cylinder',link=link)
        s.shape('torso',(0,0,1.08*q),(.27*q,.38*q,.60*q),color,link=link)
        s.shape('head',(0,0,1.57*q),(.18*q,),'skin',kind='sphere',link=link)
        if len(points)>1:
            plugin=element(model,'plugin',filename='libScriptedObstacle.so',name='sentry_simulation::ScriptedObstacle')
            element(plugin,'enabled','true')
            element(plugin,'topic','/home_large/people/enabled')
            for t,px,py,a in points:
                wp=element(plugin,'waypoint')
                element(wp,'time',t)
                element(wp,'pose',numbers((px,py,0,0,0,a)))
        metadata.append(dict(name='person_'+name,height=height,waypoints=points))
    return metadata


def write_map(s):
    resolution=.05
    origin=(-1,-1)
    width,height=1000,680
    grid=np.full((height,width),205,dtype=np.uint8)
    xs=origin[0]+(np.arange(width)+.5)*resolution
    ys=origin[1]+(height-np.arange(height)-.5)*resolution
    grid[np.ix_((ys>0)&(ys<32),(xs>0)&(xs<48))]=254
    for item in s.items:
        if not item['collision']:
            continue
        x,y,z=item['xyz']; dims=item['dims']; kind=item['kind']
        h=dims[2] if kind=='box' else (dims[1] if kind=='cylinder' else 2*dims[0])
        # Include 3 cm slippers and table legs; exclude overhanging tabletops.
        # This map is for the present low swerve platform, z in [5 mm, 0.40 m].
        if z+h/2<=.005 or z-h/2>=.40:
            continue
        extent=math.hypot(dims[0],dims[1])/2 if kind=='box' else dims[0]
        pad=resolution/math.sqrt(2)
        ix=np.flatnonzero(abs(xs-x)<=extent+pad)
        iy=np.flatnonzero(abs(ys-y)<=extent+pad)
        dx=xs[ix][None,:]-x; dy=ys[iy][:,None]-y
        if kind=='box':
            c,st=math.cos(item['yaw']),math.sin(item['yaw'])
            mask=(abs(c*dx+st*dy)<=dims[0]/2+pad)&(abs(-st*dx+c*dy)<=dims[1]/2+pad)
        else:
            # Full XY radius is conservative for a sphere slice.
            mask=dx*dx+dy*dy<=(dims[0]+pad)**2
        patch=grid[np.ix_(iy,ix)]; patch[mask]=0; grid[np.ix_(iy,ix)]=patch
    (PACKAGE/'maps/home_large.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+grid.tobytes())
    (PACKAGE/'maps/home_large.yaml').write_text(yaml.safe_dump(dict(image='home_large.pgm',mode='trinary',resolution=resolution,
        origin=[*origin,0],negate=0,occupied_thresh=.65,free_thresh=.25),sort_keys=False))


def write_plan(s, data):
    scale=23; ox=62; oy=842
    def p(x,y): return ox+x*scale,oy-y*scale
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1240" height="1010" viewBox="0 0 1240 1010">',
         '<rect width="1240" height="1010" fill="#f6f3ec"/>',
         '<g font-family="Noto Sans CJK SC, sans-serif">',
         '<text x="62" y="45" font-size="27" fill="#223d47" font-weight="bold">HOME LARGE · 单层住宅导航试验场</text>',
         f'<text x="62" y="76" font-size="15" fill="#587078">48 × 32 m / 1536 m²　｜　18 个功能房间　｜　门净宽 {min(d["clear_width"] for d in s.doors):.3f}–{max(d["clear_width"] for d in s.doors):.3f} m　｜　环路 114 m</text>']
    svg.append(f'<rect x="{ox}" y="{oy-32*scale}" width="{48*scale}" height="{32*scale}" fill="#e8e0d0"/>')
    # Draw floors first; draw overhead furniture outlines so the open space below
    # a high table is apparent without losing the domestic floor-plan context.
    items=sorted(s.items,key=lambda a: (a['collision'], a['xyz'][2]))
    for item in items:
        if item['name']=='floor': continue
        x,y,z=item['xyz']; dims=item['dims']; px,py=p(x,y)
        col=item['color']; kind=item['kind']
        if kind=='box':
            h=dims[2]
            if z-h/2>1.8: continue
            opacity='.5' if z-h/2>=.4 else '1'
            svg.append(f'<rect x="{px-dims[0]*scale/2:.2f}" y="{py-dims[1]*scale/2:.2f}" width="{dims[0]*scale:.2f}" height="{dims[1]*scale:.2f}" fill="{col}" opacity="{opacity}" transform="rotate({-math.degrees(item["yaw"]):.2f} {px:.2f} {py:.2f})"/>')
        else:
            svg.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="{dims[0]*scale:.2f}" fill="{col}"/>')
    route=' '.join(f'{p(x,y)[0]:.1f},{p(x,y)[1]:.1f}' for x,y in data['routes']['ring'])
    svg.append(f'<polyline points="{route}" fill="none" stroke="#247c88" stroke-width="3" stroke-dasharray="9 6"/>')
    for room in data['rooms']:
        x0,y0,x1,y1=room['bounds']; px,py=p((x0+x1)/2,(y0+y1)/2)
        svg.append(f'<rect x="{px-43}" y="{py-13}" width="86" height="26" rx="6" fill="#fffdf5" opacity=".92"/>')
        svg.append(f'<text x="{px}" y="{py+5}" text-anchor="middle" font-size="14" fill="#233e47">{escape(room["label"])}</text>')
        gx,gy=p(*room['goal'])
        svg.append(f'<circle cx="{gx}" cy="{gy}" r="3" fill="#247c88"/>')
    for person in data['people']:
        path=' '.join(f'{p(w[1],w[2])[0]},{p(w[1],w[2])[1]}' for w in person['waypoints'])
        svg.append(f'<polyline points="{path}" fill="none" stroke="#c86451" stroke-width="3"/>')
        _,x,y,_=person['waypoints'][0]; px,py=p(x,y)
        svg.append(f'<circle cx="{px}" cy="{py}" r="6" fill="#c86451" stroke="white" stroke-width="2"/>')
    px,py=p(*data['spawn'][:2])
    svg.append(f'<circle cx="{px}" cy="{py}" r="8" fill="#233e47" stroke="white" stroke-width="2"/>')
    svg += ['<text x="62" y="897" font-size="16" fill="#233e47">青色虚线：114 m 巡航环路　　珊瑚色：人物脚本路线　　小圆点：房间目标点</text>',
            f'<text x="62" y="927" font-size="14" fill="#587078">整车碰撞宽 {s.robot_width:.3f} m + 左右各 10 / 15 / 20 / 25 / 30 cm；门净高 2.10 m，无门槛。</text>',
            '<text x="62" y="953" font-size="14" fill="#587078">低矮拖鞋 / 散落积木 / 斜椅 / 高脚桌 / 晾衣架 / 儿童横穿。图示路线是测试任务，不代表导航已通过。</text>',
            '</g></svg>']
    (HERE/'floor_plan.svg').write_text('\n'.join(svg)+'\n')


def main():
    scene=Scene()
    rooms=architecture(scene)
    furnish(scene,rooms)
    write_xml(scene.root,HERE/'model.sdf')
    config=ET.Element('model')
    element(config,'name','Home Large - single-floor furnished residence')
    element(config,'version','1.0')
    element(config,'sdf','model.sdf',version='1.9')
    author=element(config,'author'); element(author,'name','Naturewill contributors')
    widths = sorted(set(d['clear_width'] for d in scene.doors))
    element(config,'description',f'Original 48 x 32 m home, 18 rooms, primitive collision furniture, {widths[0]:.3f}-{widths[-1]:.3f} m clear doors.')
    write_xml(config,HERE/'model.config')
    root=ET.parse(PACKAGE/'resource/worlds/home_indoor_world.sdf').getroot()
    root.set('version','1.9'); world=root.find('world')
    world.find('include/uri').text='model://home_large'
    world.find('include/name').text='home_large'
    world.find('physics/max_step_size').text='0.002'
    world.find('physics').set('name','2ms')
    data=dict(size=[48,32], robot_collision_width=scene.robot_width,door_clear_widths=widths,
              spawn=[4,10,0],rooms=rooms,doors=scene.doors,
              map_height_band=[.005,.40], map_excludes='All people; they are observed by live sensors.',
              routes={'ring':[[4,10],[46.5,10],[46.5,22],[1.5,22],[1.5,10],[4,10]]},
              people=people(scene,world),
              map_checks=[dict(xy=[2.2,2],value=0),dict(xy=[20,3.7],value=254),
                          dict(xy=[18.5,3.2],value=0),dict(xy=[4,10],value=254),dict(xy=[-.5,-.5],value=205)])
    write_xml(root,PACKAGE/'resource/worlds/home_large_world.sdf')
    (HERE/'manifest.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    write_map(scene)
    write_plan(scene,data)
    print(f'home_large: {len(scene.items)} static shapes, {len(rooms)} rooms, {len(scene.doors)} doors; clear widths {widths} m')


if __name__=='__main__':
    main()
