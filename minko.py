from skgeom import minkowski

p1 = sg.Polygon([sg.Point2(-1, -1), sg.Point2(1, -1), sg.Point2(0, 1)])
p2 = sg.Polygon([sg.Point2(3, -1), sg.Point2(5, -1), sg.Point2(5, 1), sg.Point2(3, 1)])
draw(p1, facecolor='red')
draw(p2, facecolor='blue')