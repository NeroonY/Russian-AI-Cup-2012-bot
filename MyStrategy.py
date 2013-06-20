# -*- coding: utf-8 -*-

from math import *
from FireType import FireType
from TankType import TankType
from Bonus import Bonus
from BonusType import BonusType
from Obstacle import Obstacle
import Unit


import copy


#быстрый поворот
TURN_FORW_TRACK = 1
TURN_BACK_TRACK = -1

#поворот на месте
TURN_FORW_TRACK_STATE = 0.75
TURN_BACK_TRACK_STATE = -1.0

#для легких танков
TURN_SPEED = 0.03 #pi / 140.0   #pi
SPEED_DELTA_FORW = 0.2
SPEED_DELTA_BACK = 0.1

MAX_SPEED_FORW= 3.5
MAX_SPEED_BACK= 2.5

BEGIN_STAGE = 1
BEGIN_SHOOT_STAGE = 30
FIRST_STAGE = 200
MID_STAGE = 800

SECURE_ZONE_ANGLE = pi/3


CLOSE_SHOOT_RANGE = 500

CLOSE_SECTOR = 400
MID_SECTOR = CLOSE_SECTOR*2

DANGER_RANGE = 200
SECURE_ZONE_RANGE = 200

AVG_SHELL_SPEED = 16

#зависимость ускорения от скорости
#V
table_V = [1.33, 2.12, 2.67, 3.06, 3.33, 3.52, 3.65, 3.74, 3.81, 3.85]
#a
table_A = [0.168, 0.117, 0.081, 0.057, 0.039, 0.027, 0.019, 0.013, 0.009, 0.006]

def isDeathMatchStage(world):
    return  len(getLiveEnemy(world))>3 #больше 3 живых врагов

def isLastDuelMatchStage(world):
    return  len(getLiveEnemy(world))==1 #больше 3 живых врагов

def isUnitInArea(unit, centerX, centerY, radius):
    return sqrt((unit.x - centerX)**2 + (unit.y - centerY)**2) < radius

def isDangerArea(world,x,y,radius):
    enemy = filter(lambda tank: isUnitInArea(tank, x,y,radius) , getLiveEnemy(world))
    return len(enemy) >= 2

def isFarDangerArea(me, world, x,y,radius):
    return (not isUnitInArea(me,x,y,radius)) and isDangerArea(world,x,y,radius)

def get_min_angle(me, enemy_tank):
    

    dist = me.get_distance_to_unit(enemy_tank)
    #radius = enemy_tank.width / 2.0
    radius = enemy_tank.width / 3.0
    #print dist , radius
    #c = raw_input(" ")
    return atan2(radius, dist)
    


def get_path_time(dist, angle, turn_speed, speed_delta, max_speed):
    turn_ticks = angle / turn_speed

    path_ticks = sqrt(2.0*dist / speed_delta)
    #ускоряемся не бесконечно
    max_upspeed_ticks = max_speed  / speed_delta

    # print angle , turn_ticks
    # print dist, path_ticks
    # print "==="

    #me.engine_rear_power_factor
    if (max_upspeed_ticks> path_ticks):
        return turn_ticks + path_ticks
    else:
        path_part = max_upspeed_ticks - speed_delta * (max_upspeed_ticks**2) / 2
        return turn_ticks+ max_upspeed_ticks + path_part / max_speed


def get_path_time_ext(dist, angle, turn_speed, dir_koef):
    turn_ticks = angle / turn_speed

    path_ticks = 0
    last_dist = dist
    curV = 0
    while last_dist>1:
        
        #считаем текущее ускорение
        if curV>table_V[-1]:
            curV+=table_A[-1]*dir_koef
        else:
            for i in range(0,len(table_V)):
                if curV<table_V[i]:
                    curV+=table_A[i]*dir_koef
                    break

        last_dist = last_dist - curV

        path_ticks+=1
    return turn_ticks + path_ticks
    # print angle , turn_ticks
    # print dist, path_ticks
    # print "==="

    
def get_direct_time(me, x,y):
    
    angle = fabs(me.get_angle_to(x,y))
    dist = me.get_distance_to(x,y)
    
    pt = get_path_time(dist, angle , TURN_SPEED, SPEED_DELTA_FORW, MAX_SPEED_FORW)
    pte = get_path_time_ext(dist, angle , TURN_SPEED, 1.0)
    # print pt
    # print pte
    # print "----"

    return pte


def get_back_time(me, x,y):

    angle = pi - fabs(me.get_angle_to(x,y))
    dist = me.get_distance_to(x,y)
    
    pt = get_path_time(dist, angle,  TURN_SPEED, SPEED_DELTA_BACK, MAX_SPEED_BACK )
    pte = get_path_time_ext(dist, angle,  TURN_SPEED, 0.75 )
    return pte

def get_best_time(me, x, y):
    return min(get_direct_time(me, x, y), get_back_time(me,x,y))

def test_cross(x,y, angle, unit):
    #Ax By C
    A =  -sin(angle) 

    B = cos(angle)
    C = - A*x - B*y

    p1 = A*unit.x + B*unit.y + C
    p2 = sqrt(A**2 + B**2)
    dist = fabs(A*unit.x + B*unit.y + C) / sqrt(A**2 + B**2)

    radius = sqrt (unit.width**2 + unit.height**2 ) / 2
    
    #print dist
    
    # if radius > dist:
    #     print "me", x,y, angle
    #     print "unit", unit.x, unit.y, unit.width, unit.height
    #     print "koef", A , B, C
    #     print "d r", dist , radius
    #     print "p ", p1, p2


    return radius > dist #попадание

def unitRadius(unit):
    return sqrt (unit.width**2 + unit.height**2) / 2

    
def isCloseTo(me , x, y):
    # print "isCLosetest"
    # print me.x , me.y
    # print x,y
    # print me.get_distance_to(x,y)
    # print unitRadius(me)
    # c = raw_input("")
    return me.get_distance_to(x,y) < 2*unitRadius(me)

def isTankObstacle(tank, me):
    return  tank.id != me.id and (tank.teammate or tank.crew_health ==0 or tank.hull_durability == 0)

def isEnemyLiveTank(tank):
    return  (not tank.teammate) and  tank.crew_health > 0 and tank.hull_durability > 0

#TRUE - путь свободен
def test_shell_path(me, world, dist):
    
    #передняя полуплоскость
    objs = filter(lambda x:  fabs(me.get_turret_angle_to_unit(x)) < (pi /2) , world.bonuses)
    tanks = filter(lambda x: isTankObstacle(x, me) and fabs(me.get_turret_angle_to_unit(x)) < (pi /2), world.tanks)
    obsts  = filter(lambda x:  fabs(me.get_turret_angle_to_unit(x)) < (pi /2) , world.obstacles)
    objs = objs +  tanks + obsts

    res = True
    for obj in objs:
        if me.get_distance_to_unit(obj)<dist and test_cross(me.x, me.y, me.angle  + me.turret_relative_angle, obj):
            res= False
            break

    return res

def get_turret_angle_to_unit_future(me, world, enemy):
    dist = me.get_distance_to_unit(enemy) - me.virtual_gun_length
    dist = max(dist,0)
    ticks =  dist / AVG_SHELL_SPEED #просто средняя скорость

    return me.get_turret_angle_to(enemy.x + enemy.speedX*ticks - me.speedX, enemy.y + enemy.speedY*ticks - me.speedY)


def isInWorld(world, x,y, r):
    return (x-r)>0 and (y-r)>0 and (x+r)<world.width and (y+r)<world.height

def isNearEdge(unit, world):
    radius = unit.width / 2.0* 1.1
    return (unit.x < radius) or (unit.y < radius) or (unit.x > world.width - radius) or (unit.y > world.height - radius)

def getLiveEnemy(world):
    return filter(isEnemyLiveTank, world.tanks)


def GetLineEQ(unit):
    A =  -sin(unit.angle) 
    B = cos(unit.angle)
    C = - A*unit.x - B*unit.y 
    return (A,B,C)

def GetLineIntersect((A,B, C), (A1,B1, C1)):
    if (A*B1-A1*B)< 0.01 or (B*A1-B1*A)< 0.01:
        return (-1000,-1000)


    x= (C1*B - C*B1)/(A*B1-A1*B)
    y = (C1*A - C*A1)/(B*A1-B1*A)
    return (x,y)
#находиться ли цель в широкой области охвата
def isUnderSight(tank, goal, base_angle = SECURE_ZONE_ANGLE):
    ang = tank.get_turret_angle_to_unit(goal)
    return  fabs(ang)< base_angle
    


class Evaison:
    def __init__(self,enabled = False, x =0 ,y=0,shellID=0):
        self.enabled = enabled
        self.x = x
        self.y = y
        self.shellID = shellID
class CornerMove:
    def __init__(self):
        self.enabled = False
        self.x= 0
        self.y = 0

    def activate(self, x,y):
        self.enabled = True
        self.x = x
        self.y = y

    def stop(self):
        self.enabled = False

class SecureZone:
    def __init__(self):
        self.enabled = False




class TankMoveTask:
    #def __init__(self):
    def move(self, me, world, move, tasks):
        return
        

class BaseTask(TankMoveTask):
    def move(self, me, world, move, tasks):
        return







class MyStrategy:
    def __init__(self):
        self.evaison = Evaison()
        self.cornerMove = CornerMove()
        self.secureZone = SecureZone()

    def move(self, me, world, move):
        
        self.move_stage(me, world, move )
        self.fire_stage(me, world, move )


    def select_tank(self, tank_index, team_size):
        return TankType.MEDIUM

    #return продолжать ли действия
    def doEvaison(self,  me, world, move):
        if not self.evaison.enabled:
            return False

        shells = filter(lambda s: s.id == self.evaison.shellID, world.shells)

        if len (shells)==0:
            self.evaison.enabled = False
            return False
        #пуля уже пролетела
        if fabs(shells[0].get_angle_to_unit(me))> pi/2:
            self.evaison.enabled = False
            return False




        
        # if me.get_distance_to(x,y) < 20
        #     evaison.enabled = False
        #     return False
        # print "doEvaison"
        # print me.speedX, me.speedY
        # print me.x - self.evaison.x, me.y - self.evaison.y
        # print sqrt( (me.x - self.evaison.x)**2 + (me.y - self.evaison.y)**2 ) 

        self.to_point_univ(me, world, move, self.evaison.x, self.evaison.y, True)
        return True
        # c= raw_input("")


    def move_back(self, me, world, move):
        move.right_track_power = -1.0
        move.left_track_power =  -1.0


    def getBonus(self, me, world, move):
        
        if self.secureZone.enabled:
            radius = SECURE_ZONE_RANGE            
        elif isDeathMatchStage(world):
        #     radius = world.width / 2
        # else :
        #     radius = 1.5 * world.width / 2
            radius = world.height / 3
        elif isLastDuelMatchStage(world):
            radius = 2 * world.height / 2
        else :
            radius = 1.5 * world.height / 2

        fav_bonus = []
        if me.crew_health < me.crew_max_health:
            fav_bonus.append(BonusType.MEDIKIT)
        if me.hull_durability< me.hull_max_durability*0.75:
            fav_bonus.append(BonusType.REPAIR_KIT)

        
        #bonuses = map (lambda x: (x, me.get_distance_to_unit(x)), world.bonuses )
        #отсекаем дальние       
        bonuses = filter(lambda x: me.get_distance_to_unit(x)< radius,  world.bonuses)
        #около них не должно быть много врагов
        bonuses = filter(lambda bonus: not isFarDangerArea(me, world, bonus.x, bonus.y, DANGER_RANGE), bonuses )
        #игнорим центр пока там жарко
        if isDeathMatchStage(world):
            bonuses = filter(lambda bonus: not isUnitInArea(bonus, world.width /2 , world.height /2 , world.height/4), bonuses)
        #сколько двигать
        bonuses = map (lambda x: (x, get_best_time(me, x.x, x.y)) , bonuses )
        #сорт по важности + время движения
        bonuses = sorted(bonuses, key = lambda (bonus, ticks): ( not (bonus.type in fav_bonus), ticks))

        #бонусы есть и недалеко
        if len(bonuses)>0 :
            (bonus, ticks) = bonuses[0]
            self.to_point_univ(me, world, move, bonus.x, bonus.y)
            self.stopMiscMotions()
            return True
        else:
            return False


    def checkSecureZone(self, me, world, move):
        if isLastDuelMatchStage(world):
            self.secureZone.enabled = False
        else:
            tanks = getLiveEnemy(world)
            self.secureZone.enabled = all(map(lambda tank: (not isUnderSight(tank, me)) and (not isUnitInArea(tank, me.x , me.y, SECURE_ZONE_RANGE )), tanks))

        if self.secureZone.enabled:
            self.stopMiscMotions()
            # print "zone"
            # c = raw_input("")



    def move_stage(self, me, world, move):
    	
        #c = raw_input("")
        #сразу в угол
        if world.tick <  BEGIN_STAGE:
            self.move_back(me, world,  move)
        elif world.tick < FIRST_STAGE:
            self.toCloseCorner(me, world,  move)
        else:
            
            if self.doEvaison( me, world, move):
                self.stopMiscMotions()
                return

            if self.detect_danger(me,world,move):
                self.stopMiscMotions()
                return

            self.checkSecureZone(me,world, move)
            if self.getBonus(me, world, move):
                self.stopMiscMotions()
                return        
            else:
                #валим в угол
                self.toCloseCorner(me, world,  move)


    def evacuateTo(self, x,y, shellID, me, world,  move):
        
        self.evaison = Evaison(True, x,y, shellID)
        self.doEvaison(me, world,  move)



        
    def detect_danger(self, me, world,  move):

        SMALL_SECTOR = pi / 6

        shells = filter(lambda x: fabs(x.get_angle_to_unit(me))< SMALL_SECTOR , world.shells)
        if len(shells) > 0:
            for shell in shells:
                if test_cross(shell.x, shell.y, shell.angle, me) :
                    time_remain = (shell.get_distance_to_unit(me) - me.height / 2) / AVG_SHELL_SPEED
                    
                    hope_angle = pi / 12
                    ang = me.get_angle_to_unit(shell)
                    #не можем сбежать с линии
                    if fabs(ang) < hope_angle or (pi - fabs(ang) < hope_angle):
                        continue


                    #точки эвакуации по кругу от текущего положения
                    r = me.height  * 3#для гарантии
                    esc_points= []
                    #боковины не работают, нетак расчет
                    # for i in range(1,4)+range(5,8):
                    #     ang = shell.angle + pi/4.0 *i
                    speed = me.speedX * cos(me.angle) + me.speedY * cos(pi/2 - me.angle)
                    if speed > -1:
                        esc_points.append((me.x+ r*cos(me.angle), me.y + r * sin(me.angle) ))
                    else:
                        esc_points.append((me.x+ r*cos(me.angle+pi), me.y + r * sin(me.angle+pi) ))

                    self.evacuateTo(esc_points[0][0],esc_points[0][1], shell.id, me, world,  move)
                    return True
                    
                                        #print "speed", speed
                    # for i in [0]:
                    #     ang = me.angle + pi*i

                    #     x = me.x + r * cos(ang)
                    #     y = me.y + r * sin(ang)
                    #     new_me = copy.copy(me)
                    #     new_me.x = x
                    #     new_me.y = y
                    #     #проверка новых точек
                    #     if not test_cross(shell.x, shell.y, shell.angle, new_me):
                    #         esc_points.append((x,y))
                    
                    #не вылезаем за пределы экрана
                    # esc_points = filter(lambda (x,y): isInWorld(world, x,y, me.height), esc_points)
                    # if len (esc_points)>0:
                    #     esc_points = map(lambda (x,y):  ( (x,y), get_best_time(me, x,y)), esc_points)

                    #     esc_points = sorted(esc_points, key = lambda (xy, time): time );
                    #     if esc_points[0][1]<time_remain:                            
                    #         # print "new evaison"
                    #         (x,y) = esc_points[0][0]
                    #         self.evacuateTo(x,y, shell.id, me, world,  move)
                    #         return True
        return False
                    

    def fire_stage(self, me, world, move):
    	#move.turret_turn = pi
        #move.fire_type = FireType.PREMIUM_PREFERRED
        

        all_tanks = filter( isEnemyLiveTank, world.tanks) 
    	
    	
    	if len(all_tanks) == 0:
            move.fire_type = FireType.NONE
            return

        tanks_data = map(lambda x: (x, get_turret_angle_to_unit_future(me, world, x), me.get_distance_to_unit(x)) , all_tanks)
        tanks_data = sorted(tanks_data, key = lambda (tank, ang, dist) : fabs(ang))
        #если навелись и готовы стрелять - бьем, одновремено крутим к следующему
        already_shoot = False
        (sel_tank, angle_to_enemy, dist)= tanks_data[0]
        min_angle = get_min_angle(me, sel_tank)

        if me.remaining_reloading_time == 0  and fabs(angle_to_enemy) < min_angle and test_shell_path(me, world, me.get_distance_to_unit(sel_tank)):
            if world.tick >= BEGIN_SHOOT_STAGE:
                already_shoot = True
                if me.get_distance_to_unit(sel_tank) < CLOSE_SHOOT_RANGE:
                    move.fire_type = FireType.PREMIUM_PREFERRED
                else:
                    move.fire_type = FireType.REGULAR 
        
        #теперь смотрим все
        #сначала ближние танки
        close_tanks = filter(lambda (x,ang,dist): dist< CLOSE_SECTOR, tanks_data) 
        mid_tanks = filter(lambda (x,ang,dist): dist< MID_SECTOR, tanks_data) 
        if len(close_tanks)>0:
            tanks_data = sorted(close_tanks, key = lambda (tank, ang, dist) : fabs(ang))
        elif len(mid_tanks)>0:
            tanks_data = sorted(mid_tanks, key = lambda (tank, ang, dist) : fabs(ang))
        else:
            tanks_data = sorted(tanks_data, key = lambda (tank, ang, dist) : fabs(ang))

        (sel_tank, angle_to_enemy, dist)= tanks_data[0]
        min_angle = get_min_angle(me, sel_tank)

        # print min_angle
        # print angle_to_enemy
        # print "-------------"


        if (angle_to_enemy > min_angle):
            move.turret_turn = 1
            #если нет других дествий подворачиваем и сам танк
            if move.left_track_power ==0 and move.right_track_power == 0 :
                move.left_track_power = TURN_FORW_TRACK_STATE
                move.right_track_power = TURN_BACK_TRACK_STATE 

        elif (angle_to_enemy < -min_angle):
            move.turret_turn = -1
            if move.left_track_power ==0 and move.right_track_power == 0 :
                move.left_track_power = TURN_BACK_TRACK_STATE 
                move.right_track_power = TURN_FORW_TRACK_STATE
        else:
            move.turret_turn = angle_to_enemy
            if world.tick >= BEGIN_SHOOT_STAGE:
                if (not already_shoot) and test_shell_path(me, world, me.get_distance_to_unit(sel_tank)):
                    #если стреляем вдоль оси враж танка, дальность для премиума больше
                    enemy_rotate_ang = fabs(sel_tank.get_angle_to_unit(me))
                    min_rot_ang = pi / 10
                    if  enemy_rotate_ang < min_rot_ang or (pi - enemy_rotate_ang) < min_rot_ang:
                        close_shoot_range = 2*CLOSE_SHOOT_RANGE
                        # print "line"
                        # c = raw_input("")
                    else:
                        close_shoot_range = CLOSE_SHOOT_RANGE

                    if me.get_distance_to_unit(sel_tank) < close_shoot_range:
                        move.fire_type = FireType.PREMIUM_PREFERRED
                    else:
                        move.fire_type = FireType.REGULAR
        




    def to_point(self, me, world, move, x,y):
    	if me.get_distance_to(x,y) < (me.width / 3):
            move.left_track_power = 0
            move.right_track_power = 0
            return

    	ang = me.get_angle_to(x,y)
    	MIN_ANGLE = pi / 20
        MED_ANGLE = pi / 4
        TURN_KOEF = 0.5

        if (ang > MIN_ANGLE) :# если угол сильно положительный,
            if isCloseTo(me , x, y):
                move.left_track_power = TURN_FORW_TRACK_STATE
                move.right_track_power = TURN_BACK_TRACK_STATE
            elif (ang > MED_ANGLE) :                 #если сильно поворачивать, то разворот 
                move.left_track_power = TURN_FORW_TRACK
                move.right_track_power = TURN_BACK_TRACK
            else :
                move.left_track_power = TURN_FORW_TRACK
                move.right_track_power = TURN_KOEF * TURN_FORW_TRACK * cos(ang)
        elif (ang < -MIN_ANGLE) :
            if isCloseTo(me , x, y):
                    move.left_track_power = TURN_BACK_TRACK_STATE
                    move.right_track_power = TURN_FORW_TRACK_STATE
            elif (fabs(ang) > MED_ANGLE) : 
                    move.left_track_power = TURN_BACK_TRACK
                    move.right_track_power = TURN_FORW_TRACK
            else :
                move.left_track_power = TURN_KOEF * TURN_FORW_TRACK * cos(ang)
                move.right_track_power = TURN_FORW_TRACK
        else : #просто вперед
            move.right_track_power = 1.0
            move.left_track_power = 1.0

    def to_point_back(self, me, world ,move, x,y, curved = False):
        
        if me.get_distance_to(x,y) < (me.width / 3):
            move.left_track_power = 0
            move.right_track_power = 0
            return

        ang = me.get_angle_to(x,y)
        MIN_ANGLE = pi - pi / 20
        
        if curved:
            MED_ANGLE = pi - pi / 3
        else:
            MED_ANGLE = pi - pi / 4
        

        TURN_KOEF = 0.5


        if ( 0 < ang < MIN_ANGLE) : # если угол сильно положительный,           
            if isNearEdge(me, world) or isCloseTo(me , x, y):
                move.left_track_power = TURN_BACK_TRACK_STATE
                move.right_track_power = TURN_FORW_TRACK_STATE
            elif ang < MED_ANGLE:
                move.left_track_power = TURN_BACK_TRACK
                move.right_track_power = TURN_FORW_TRACK 
            else:
                move.left_track_power = TURN_BACK_TRACK
                move.right_track_power = TURN_BACK_TRACK * cos(pi - ang) *TURN_KOEF
        elif (-MIN_ANGLE < ang < 0):
            if isNearEdge(me, world) or isCloseTo(me , x, y):                
                move.left_track_power =  TURN_FORW_TRACK_STATE
                move.right_track_power = TURN_BACK_TRACK_STATE
            elif (-MED_ANGLE < ang ):
                move.left_track_power =  TURN_FORW_TRACK
                move.right_track_power = TURN_BACK_TRACK             
            else:
                move.left_track_power =  TURN_BACK_TRACK * cos(pi - ang) *TURN_KOEF
                move.right_track_power = TURN_BACK_TRACK   
        else:
            move.right_track_power = -1.0
            move.left_track_power = -1.0
    
    def to_point_univ(self, me, world, move, x,y, strong_close= False):
        
        if strong_close:
            close_radius = 20
        else:
            close_radius = (me.width / 3) 

        if me.get_distance_to(x,y) < close_radius :
            move.left_track_power = 0
            move.right_track_power = 0
            return

        dir_time = get_direct_time(me, x,y)
        back_time = get_back_time(me,x,y)

        
        # print dir_time
        # print back_time
        # print "-------"
        # c= raw_input("")

        if dir_time < back_time:
            self.to_point(me, world, move, x,y)
        else:
            self.to_point_back(me, world, move, x,y)

        
    def stopMiscMotions(self):
        self.cornerMove.stop()


    def toCloseCorner(self, me ,world, move):
        
        if self.secureZone.enabled:
            return

        #если уже выбран движемся к нему
        if self.cornerMove.enabled:
            x = self.cornerMove.x
            y = self.cornerMove.y
        else:

            
            if world.tick < FIRST_STAGE:
                delta = me.height 
            else:
                delta = 120

                
            if me.x > (world.width / 2):
                x = world.width - delta
            else:
                x = delta

            if me.y > (world.height / 2):
                y = world.height - delta
            else:
                y = delta

            #print "corner", x,y
            self.cornerMove.activate(x,y);

        if world.tick < FIRST_STAGE:
            #self.to_point_back(me, world, move,x,y,True)
            self.to_point_back(me, world, move,x,y)
        else:
            self.to_point_univ(me, world, move, x,y)
#        self.to_point(me, move, x,y)

        # print world.tick, ":to_corner",x,y
        # c= raw_input(" " ) 

    	

    		
     