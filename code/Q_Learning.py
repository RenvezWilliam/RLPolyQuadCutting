import random
from math import *
from numpy import zeros,array
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from pickle import *
import os.path
import time

import gmsh

""""global value to indicate how to round floating numbers"""
nb_digit_rounding = 4


def initialize():
    """ Initialize the gmsh module"""
    gmsh.initialize()
    gmsh.clear()
    # to avoid messages on the console
    gmsh.option.setNumber("General.Verbosity", 0)
    # gmsh.option.setNumber("Mesh.Algorithm", 3) # minimal delaunay
    # 11 <- quad frame based


def create_rectangle(x, y, dx, dy):
    """ create a 2D rectangle from left bottom corner (x,y) and
    side size dx in X direction and dy in Y direction.
    dx and dy must be strictly positive"""
    assert dx > 0
    assert dy > 0
    return gmsh.model.occ.add_rectangle(x, y, 0, dx, dy)


def fuse(pieces):
    """ fuses the input given list of pieces to return the
    Boolean union of all of those pieces """
    tool = [(2, pieces[0])]
    objs = []
    for i in range(1, len(pieces)):
        objs.append((2, pieces[i]))
    op = gmsh.model.occ.fuse(objs, tool)
    gmsh.model.occ.synchronize()
    return op[0][0][1]


def get_cell_tags(dim):
    info = gmsh.model.occ.getEntities(dim)
    tags = []
    for i, j in info:
        tags.append(j)
    return tags


def get_point_tags():
    """" returns the tags of all the vertices"""
    return get_cell_tags(0)


def get_edge_tags():
    """" returns the tags of all the edges"""
    return get_cell_tags(1)


def get_face_tags():
    """" returns the tags of all the faces"""
    return get_cell_tags(2)


def get_end_points(curve_tag):
    """ returns the two end points (extremities) of an edge"""
    data = gmsh.model.getAdjacencies(1, curve_tag)
    return data[1]


class Query:
    def __init__(self):
        self.__points = get_point_tags()
        self.__edges = get_edge_tags()
        self.__l2p = []

    def adjacent_curves(self, point_tag):
        curves = []
        for adj in self.__l2p:
            if adj[1] is point_tag:
                curves.append(adj[0])
        return curves

    def adjacent_points(self, curve_tag):
        points = []
        for adj in self.__l2p:
            if adj[0] is curve_tag:
                points.append(adj[1])
        return points

    def get_curves(self, face_tag):
        return gmsh.model.occ.get_curve_loops(face_tag)[1][0]

    def get_corners(self, face_tag):
        curves = gmsh.model.getAdjacencies(2, face_tag)[1]
        all_points = get_point_tags()
        corners = []
        for i_curve in range(0, len(curves)):
            j_index = i_curve - 1
            if i_curve == 0:
                j_index = len(curves) - 1
            prev_pnts = get_end_points(curves[j_index])
            current_pnts = get_end_points(curves[i_curve])
            common_pnt = 0
            if prev_pnts[0] == current_pnts[0]:
                common_pnt = current_pnts[0]
            elif prev_pnts[0] == current_pnts[1]:
                common_pnt = current_pnts[1]
            elif prev_pnts[1] == current_pnts[0]:
                common_pnt = current_pnts[0]
            else:
                common_pnt = current_pnts[1]
            corners.append(common_pnt)

        return corners


def point_coordinate(tag):
    """:return (x,y) the location of the point 'tag'"""
    bb = gmsh.model.occ.getBoundingBox(0, tag)
    return (round(bb[0], nb_digit_rounding),
            round(bb[1], nb_digit_rounding))


def cut(point_tag, face_tag, dir):
    """ Cut the face face_tag starting from point-tag in direction dir
        dir can be 1 (north), 2 (south), 3 (east), 4 (west)  """
    to_infinity = 0
    coord = point_coordinate(point_tag)
    if dir == 1:
        to_infinity = gmsh.model.occ.add_point(coord[0], coord[1] + 100, 0)
    elif dir == 2:
        to_infinity = gmsh.model.occ.add_point(coord[0], coord[1] - 100, 0)
    elif dir == 3:
        to_infinity = gmsh.model.occ.add_point(coord[0] + 100, coord[1], 0)
    elif dir == 4:
        to_infinity = gmsh.model.occ.add_point(coord[0] - 100, coord[1], 0)

    cut_edge = gmsh.model.occ.addLine(point_tag, to_infinity)
    gmsh.model.occ.fragment([(2, face_tag)], [(1, cut_edge)])
    gmsh.model.occ.synchronize()
    for e in get_edge_tags():
        if len(gmsh.model.getAdjacencies(1, e)[0]) == 0:
            gmsh.model.occ.remove([(1, e)], True)
    gmsh.model.occ.synchronize()
    return to_infinity


def finalize():
    """ Finalize the gmsh call """
    gmsh.fltk.run()
    gmsh.finalize()


def center(face_tag):
    return gmsh.model.occ.getCenterOfMass(2, face_tag)


def area(face_tag):
    return gmsh.model.occ.getMass(2, face_tag)


def remesh():
    gmsh.model.occ.synchronize()
    gmsh.model.mesh.clear([])
    gmsh.model.mesh.generate(2)  # gives the coarse mesh only


def print_infos():
    print("---------------------------------")
    print("Faces ", get_face_tags())
    for f_tag in get_face_tags():
        q = Query()
        print("Face ", f_tag, ", center ", center(f_tag), ", and area ", area(f_tag))
        print(" --> curves ", q.get_curves(f_tag))
        print(" --> points ", q.get_corners(f_tag))


# === Mes ajouts === #

def is_a_rectangle(f_tag, print_info=False):
    q = Query()

    points = q.get_corners(f_tag)

    min_x = point_coordinate(points[0])[0]
    min_y = point_coordinate(points[0])[1]
    max_x = point_coordinate(points[0])[0]
    max_y = point_coordinate(points[0])[1]

    # Obtiens les valeurs minimums et maximums de x et y

    for p in points:
        if min_x > point_coordinate(p)[0]:
            min_x = point_coordinate(p)[0]
        if min_y > point_coordinate(p)[1]:
            min_y = point_coordinate(p)[1]
        if max_x < point_coordinate(p)[0]:
            max_x = point_coordinate(p)[0]
        if max_y < point_coordinate(p)[1]:
            max_y = point_coordinate(p)[1]

    # Maintenant qu'on à les valeurs minimums, on regarde si tous les points de la figure contiennent au moins une des
    # valeurs, si ce n'est pas le cas, ce n'est pas un rectangle.

    if print_info:
        print("-=-=-= Info =-=-=-")
        print("f_tag :", f_tag)
        print("min_x :", min_x)
        print("min_y :", min_y)
        print("max_x :", max_x)
        print("max_y :", max_y)
        print("------------")
        for p in points:
            print("point trouvé à", point_coordinate(p))

    for p in points:
        if min_x == point_coordinate(p)[0] or min_y == point_coordinate(p)[1] or max_x == point_coordinate(p)[0] or max_y == point_coordinate(p)[1]:
            continue
        return False

    return True


def is_on_a_face(point, direction, f_tag):
    q = Query()
    point_test = None
    coord = point_coordinate(point)
    if direction == 1:
        point_test = (coord[0], coord[1] + 0.5)
    elif direction == 2:
        point_test = (coord[0], coord[1] - 0.5)
    elif direction == 3:
        point_test = (coord[0] + 0.5, coord[1])
    elif direction == 4:
        point_test = (coord[0] - 0.5, coord[1])

    points = q.get_corners(f_tag)

    coords = []
    for p in points:
        coords.append(point_coordinate(p))

    point = Point(point_test)
    polygon = Polygon(coords)

    return polygon.covers(point), polygon.contains(point)


def create_shape(file):
    # créer un tableau avec les différents rectangle inscrit sur le fichier et transforme le tout en int
    lines = file.read().split('\n')
    for i in range(len(lines)):
        lines[i] = lines[i].split(',')
        for j in range(len(lines[i])):
            lines[i][j] = int(lines[i][j])
    # print(lines)
    # Créer tous les rectangles à partir du tableau obtenu juste avant
    rectangles = []
    for i in range(len(lines)):
        rectangles.append(create_rectangle(lines[i][0], lines[i][1], lines[i][2], lines[i][3]))
    del lines
    return fuse(rectangles)


def algo():
    # Récupère la surface initiale
    faces = get_face_tags()

    while faces:
        f = random.choice(faces)  # Prend une face aléatoirement
        points_candidats = get_cutable_pts(f)  # Récupère les points candidats pour être coupé
        p = random.choice(points_candidats)  # Récupère un point
        c = random.choice(can_cut(p, f))  # Récupère une découpe possible

        cut(p, f, c)  # Fait la découpe

        faces = get_all_non_square_faces()


def get_all_non_square_faces():
    values = []
    faces = get_face_tags()
    print("-=-=-=-=-=-")
    for f in faces:
        print(f, "is a rectangle :", is_a_rectangle(f))
        if not is_a_rectangle(f):
            values.append(f)
    return values


def get_cutable_pts(f_tag):
    q = Query()
    pts_list = q.get_corners(f_tag)
    values = []

    for p in pts_list:
        if can_cut(p, f_tag):
            values.append(p)

    return values


def launch():
    algo()
    print("score =", get_score())

def get_x(point_tag):
    # renvoi la coordoné sur x d'un point
    no_parametrization = []
    [x, y, z] = gmsh.model.getValue(0, point_tag, no_parametrization)
    return x


def get_y(point_tag):
    # renvoi la coordoné sur y d'un point
    no_parametrization = []
    [x, y, z] = gmsh.model.getValue(0, point_tag, no_parametrization)
    return y


def get_z(point_tag):
    # renvoi la coordoné sur z d'un point
    no_parametrization = []
    [x, y, z] = gmsh.model.getValue(0, point_tag, no_parametrization)
    return z


def get_dist(point_tag1, point_tag2):
    # renvoi la distance entre 2 points

    return sqrt((get_x(point_tag1) - get_x(point_tag2)) ** 2 + (get_y(point_tag1) - get_y(point_tag2)) ** 2)


def get_dir(point_tag1, point_tag2):
    # renvoi la direction d'un pooint 2 par rapport a un point 1
    """dir = 1 = nord | 2 = sud | 3 = est | 4 = ouest"""

    no_parametrization = []
    [x1, y1, z1] = gmsh.model.getValue(0, point_tag1, no_parametrization)

    no_parametrization = []
    [x2, y2, z2] = gmsh.model.getValue(0, point_tag2, no_parametrization)

    if y2 > y1:
        return 1

    if y1 > y2:
        return 2

    if x2 > x1:
        return 3

    if x1 > x2:
        return 4

    return False


def get_score():
    q = Query()
    ratio = 0
    for f_tag in get_face_tags():
        points = q.get_corners(f_tag)

        ratio = ratio + abs(get_dist(points[0], points[1]) - get_dist(points[1], points[2]))

    ratio = ratio / len(get_face_tags())

    return ratio


def can_cut(point, f_tag):
    dir = []
    for i in range(4):
        j = i + 1
        if is_on_a_face(point, j, f_tag)[1]:
            dir.append(j)
    return dir

def get_ref_point(point, ListFace, c):

    """
    revoie a un tableaux contenant:

        - avec longueur = longueur du rectangle englobant toute les face de Face et largeur = largeur du rectangle englobant toute les face de ListFace
                si longueur  > largeur => 1;  si longueur  < largeur => 2;    si longueur  = largeur => 3

        - liste des directions ou il est possible de couper

        - direction du centre fonction center
    """
    q = Query()
    ref = []
    FinalRef = ""
    l = get_encompassing(ListFace)
    Face = None

    if l[0] == l[1]:
        ref.append(3)
    elif l[0] < l[1]:
        ref.append(2)
    else:
        ref.append(1)

    for f in ListFace:
        if Face != None:
            break
        for p in q.get_corners(f):
            if p == point:
                Face = f
                break
    ref.append(can_cut(point, Face))

    Point_x = get_x(point)
    Point_y = get_y(point)

    dir = []

    if c[1] > Point_y:
        dir.append(1)

    if Point_y > c[1]:
        dir.append(2)

    if c[0] > Point_x:
        dir.append(3)

    if Point_x > c[0]:
        dir.append(4)

    ref.append(dir)

    for r in ref:
        FinalRef = FinalRef + str(r) + "/"

    return FinalRef

def get_encompassing(ListFace):

    """
    Renvoie la longeur et la largeur du rectangle englobant toute les face de ListFace
    """

    if len(ListFace) == 0:
        return False

    q = Query()
    point = q.get_corners(ListFace[0])[0]
    Min_x = get_x(point)
    Min_y = get_y(point)
    Max_x = get_x(point)
    Max_y = get_y(point)

    for f in ListFace:
        for p in q.get_corners(f):
            if get_x(p) > Max_x:
                Max_x = get_x(p)
            if get_y(p) > Max_y:
                Max_y = get_y(p)
            if get_x(p) < Min_x:
                Min_x = get_x(p)
            if get_y(p) < Min_y:
                Min_y = get_y(p)

    return [Max_x - Min_x, Max_y - Min_y]

def Q_Learning_train_random(ScoreMin, NBTest, gain, shape):#RatioMin, NBegal, gain, PoidInitial):

    Data = {}
    #random.seed(2)

    nbbcl = 0
    q = Query()

    for s in shape:
        for nb in range(NBTest):
            gmsh.clear()
            print("Clear")

            file_name = s + ".txt"
            f = open(file_name, "r")
            ps = create_shape(f)
            f.close()


            c = center( get_face_tags()[0])

            ListeAction = {}

            Face = get_all_non_square_faces()

            print("Début preocess: ", nb)

            while len(Face) != 0: #and nbbcl != 10:
                nbbcl += 1
                i = 0

                #print(len(get_face_tags()))

                #séléction de la face parmis celle qui n'on pas 4 points en fonction de Data

                #séléction random
                if len(Face) == 1:
                    r = 0
                else:
                    r = random.randint(0, (len(Face) - 1))

                points = q.get_corners(Face[r])
                f_tag = Face[r]

                print(len(Face), " face a découper")
                print("selection face: ", f_tag)

                #selection aléatoire du point

                PointValide = []

                for p in points:
                    if len(can_cut(p, f_tag)) != 0:
                        PointValide.append(p)

                if len(PointValide) == 1:
                    r = 0
                else:
                    r = random.randint(0, len(PointValide) - 1)

                p = PointValide[r]

                print("selection point: ", p)

                ref = get_ref_point(p, Face, c)

                if not ref in Data:
                    Data[ref] = [0,0,0,0]
                if not ref in ListeAction:
                    ListeAction[ref] = [0,0,0,0]

                #selection de la direction a cut

                CanCut = can_cut(p, f_tag)

                if len(CanCut) == 1:
                    r = 0
                else:
                    r = random.randint(0, len(CanCut) - 1)

                cut(p, f_tag, CanCut[r])

                print("cut: ", CanCut[r])

                ListeAction[ref][CanCut[r] - 1] +=1

                Face = get_all_non_square_faces()

            if get_score() <= ScoreMin:

                print("Gagner, ", get_score(), " < ", ScoreMin)

                for ref in ListeAction.keys():
                    for i in range(4):
                        Data[ref][i] += ListeAction[ref][i] * gain
            else:

                print("Perdu, ", get_score(), " > ", ScoreMin)

            print("Fin preocess: ", nb)
        print("Fin calcul: ", s)
    return Data

def Q_Learning_execute(Data, shape):
    gmsh.clear()
    print("Clear")

    file_name = shape + ".txt"
    f = open(file_name, "r")
    ps = create_shape(f)
    f.close()

    c = center(get_face_tags()[0])

    Face = get_all_non_square_faces()

    while len(Face) != 0:  # and nbbcl != 10:
        # séléction de la face parmis celle qui n'on pas 4 points en fonction de Data

        # séléction random
        if len(Face) == 1:
            r = 0
        else:
            r = random.randint(0, (len(Face) - 1))

        points = q.get_corners(Face[r])
        f_tag = Face[r]

        print(len(Face), " face a découper")
        print("selection face: ", f_tag)

        # selection aléatoire du point

        PointValide = []

        for p in points:
            if len(can_cut(p, f_tag)) != 0:
                PointValide.append(p)

        if len(PointValide) == 1:
            r = 0
        else:
            #selectionne le point dont la somme des poids est la plus élevé
            MaxSomme = [-1, 0]
            for p in PointValide:
                ref = get_ref_point(p, Face, c)
                if ref in Data:
                    somme = Data[ref][0] + Data[ref][1] + Data[ref][2] + Data[ref][3]

                    if somme > MaxSomme[1]:
                        MaxSomme[0] = p
                        MaxSomme[1] = somme
            if MaxSomme[0] == -1:
                r = random.randint(0, len(PointValide) - 1)
            else:
                for i in range(len(PointValide)):
                    if p == MaxSomme[0]:
                        r = i

        p = PointValide[r]

        print("selection point: ", p)

        ref = get_ref_point(p, Face, c)

        # selection de la direction a cut

        if not ref in Data:
            CanCut = can_cut(p, f_tag)
            dir = CanCut[random.randint(0, len(CanCut) - 1)]
        else:
            max = 0
            dir = -1
            for i in range(4):
                if Data[ref][i] > max:
                    max = Data[ref][i]
                    dir = i+1
            if dir == -1:
                CanCut = can_cut(p, f_tag)
                dir = CanCut[random.randint(0, len(CanCut) - 1)]

        cut(p, f_tag, dir)

        print("cut: ", dir)

        Face = get_all_non_square_faces()

    print("Fin Execute, score =", get_score())

#######################################################

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # first, we initialize the gmsh environment
    initialize()

    # ======================================================
    # Create a first shape by assembling rectangles
    # ======================================================

    """ r1 = create_rectangle(0, 0, 10, 10)
    r2 = create_rectangle(5, 5, 10, 3)
    r3 = create_rectangle(-5, 0, 7, 2)
    ps = fuse([r1, r2, r3])"""

    #RatioMin, NBegal, gain, PoidInitial

    #Data = Q_Learning_train(3, 5, 10,50)

    #Q_Learning_Execute(Data)

    """q = Query()
    for f_tag in get_face_tags():

        for p in q.get_corners(f_tag):

            print("Point: ", p, " direction: ", can_cut(p, f_tag))"""

    q = Query()

    """file_name = "shape.txt"
    f = open(file_name, "r")
    ps = create_shape(f)
    f.close()"""

    shape = []
    for i in range(1, 31):
        shape.append("shape_" + str(i))

    l = range(100000)

    tps1 = time.clock()
    random.shuffle(l)

    Data = Q_Learning_train_random(4, 100, 10, shape)

    tps2 = time.clock()
    print("temp d'execution: ", tps2 - tps1, "s")

    i = 1
    while os.path.isfile("Data\data_" + str(i) + ".txt"):
            i += 1

    f = open("Data\data_" + str(i) , "wb")
    dump(Data, f)
    f.close()

    #Q_Learning_execute(Data,"shape")

    #launch()

    """Face = get_all_non_square_faces()

    for f in Face:
        print("face: ", f)
        for p in q.get_corners(f):
            print(p, ", ", can_cut(p, f))"""

    remesh()

    gmsh.write("mesh_gmsh.vtk")
    finalize()

