import json


def euler_distance(p1,p2):
    return ((p2['x']-p1['x'])*(p2['x']-p1['x']) + (p2['y']-p1['y'])*(p2['y']-p1['y']))**0.5

def point_input():
    x = float(input('Enter x'))
    y = float(input('Enter y'))
    flip = input('Flip?')
    return {'x':x,'y':y,'flip':flip=='y'}

def path_input():
    cont = 'y'
    path =[]
    while cont=='y':
        point = point_input()
        if point['flip']:
            ptemp = dict(point)
            ptemp['flip'] = False
            path.append(ptemp)
            ptemp = dict(point)
            ptemp['flip'] = True
            path.append(ptemp)
            ptemp = dict(point)
            ptemp['flip'] = True
            path.append(ptemp)
        else:
            path.append(point)
        cont = input('Enter more points?')
    return path

def give_between_points(p1,p2,increment):
    distance = euler_distance(p1,p2)
    no_of_between_points = int(distance//increment)
    path = []
    for i in range(no_of_between_points):
        m = increment*(i+1)
        n = distance - m
        path.append({'x':(m*p2['x']+n*p1['x'])/(m+n),'y':(m*p2['y']+n*p1['y'])/(m+n),'flip':False})
    return path


def main():
    cont = 'y'
    full_path = []
    while cont=='y':
        robot_id = int(input('Enter robot ID'))
        user_path = path_input()
        increment = float(input('Enter increment'))
        path_for_robot = []
        for i in range(1,len(user_path)):
            path_for_robot.append(user_path[i-1])
            path_for_robot = path_for_robot + give_between_points(user_path[i-1],user_path[i],increment)
        path_for_robot.append(user_path[-1])
        print('Enter coordinates of Other bots')
        static_coordinates = dict()
        for j in range(4):
            if robot_id!=j+1:
                print('For robot_id'+str(j+1))
                static_coordinates['r_'+str(j+1)] = point_input()
        for point in path_for_robot:
            four_bot_point = dict()
            for j in range(4):
                if robot_id==j+1:
                    four_bot_point['r_'+str(j+1)] = point
                else:
                    four_bot_point['r_'+str(j+1)] = static_coordinates['r_'+str(j+1)]
            full_path.append(four_bot_point)
        cont = input('Enter more points?')
    
    with open('path.json','w') as f:
        json.dump(full_path,f)


if __name__=='__main__':
    main()


        
