from math import sin, cos, pi

def main(start: tuple, radius: float, rotation_direction=1, path="circle.dat", no_points=100):
    points = list()

    for i in range(100):
        point = start[0] + radius*(cos(rotation_direction*2*pi*i/no_points)-1), \
                start[1] + radius*sin(rotation_direction*2*pi*i/no_points)
        points.append(' '.join((str(point[0]), str(point[1]))))

    with open(path, 'w') as f:
        f.write(str(no_points) + '\n')
        f.write('\n'.join(points))
        f.write('\n')

if __name__=="__main__":
    main((0,0), 2, -1, "unitcircle.dat", 30)
