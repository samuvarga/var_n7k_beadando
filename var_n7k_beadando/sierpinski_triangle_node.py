import turtle

def draw_triangle(points, color, t):
    t.fillcolor(color)
    t.up()
    t.goto(points[0])
    t.down()
    t.begin_fill()
    t.goto(points[1])
    t.goto(points[2])
    t.goto(points[0])
    t.end_fill()

def sierpinski(points, degree, t):
    colors = ['blue', 'red', 'green', 'yellow', 'purple', 'orange']
    draw_triangle(points, colors[degree % len(colors)], t)
    
    if degree > 0:
        mid1 = midpoint(points[0], points[1])
        mid2 = midpoint(points[1], points[2])
        mid3 = midpoint(points[2], points[0])
        
        sierpinski([points[0], mid1, mid3], degree - 1, t)
        sierpinski([mid1, points[1], mid2], degree - 1, t)
        sierpinski([mid3, mid2, points[2]], degree - 1, t)

def midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

def main():
    screen = turtle.Screen()
    screen.bgcolor("white")
    
    t = turtle.Turtle()
    t.speed(0)
    
    size = 300
    initial_points = [(-size, -size / 2), (0, size), (size, -size / 2)]
    
    depth = 4
    sierpinski(initial_points, depth, t)
    
    t.hideturtle()
    screen.mainloop()

if __name__ == "__main__":
    main()
