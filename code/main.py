import gmsh
from math import *

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


# ===Mes ajouts===#

def is_on_a_face(point, direction):

    point_test = None
    coord = point_coordinate(point)
    if direction == 1:
        point_test = (coord[0], coord[1] + 0.5)
    elif direction == 2:
        point_test = (coord[0], coord[1] - 0.5)
    elif direction == 3:
        point_test = (coord[1] + 0.5, coord[1])
    elif direction == 4:
        point_test = (coord[1] - 0.5, coord[1])

    surfaces = get_face_tags()

    return True

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


def glouton():
    all_faces_has_four_points = True
    q = Query()
    last_value = None

    for f_tag in get_face_tags():

        if len(q.get_corners(f_tag)) == 4:
            continue

        all_faces_has_four_points = False

        corners = q.get_corners(f_tag)
        # print("points : ", corners, " | f_tag : ", f_tag)
        for corner in corners:
            dir_to_cut = can_cut(corner, f_tag)
            # print("Point :", corner, "Face :", f_tag, "directions possible :", dir_to_cut)
            if len(dir_to_cut) == 0:
                continue
            # print("cuting", "point :", corner, "dir :", dir_to_cut[0])
            cut(corner, f_tag, dir_to_cut[0])

            last_value = (corner, dir_to_cut, f_tag)

            break
        break

    return all_faces_has_four_points, last_value


def launch():
    test = False
    last_val = None

    """for i in range(10):
        glouton()"""

    while not test:
        test, last = glouton()

        if last == last_val:
            test = True
            print("Erreur, n'a pas coupé vers la direction", last_val)
        last_val = last

    print("score =", get_score())
    # print(is_on_a_face(11, 1))

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
    #renvoi la distance entre 2 points

    return sqrt((get_x(point_tag1)-get_x(point_tag2))**2 + (get_y(point_tag1)-get_y(point_tag2))**2)

def get_dir(point_tag1, point_tag2):
    #renvoi la direction d'un pooint 2 par rapport a un point 1
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
    # En utilisant le point et le f_tag, on cherche à voir s'il y a une curve adjaçant au point dans la même direction.
    """dir = 1 = nord | 2 = sud | 3 = est | 4 = ouest"""
    q = Query()
    all_curves = q.get_curves(f_tag)
    c_point = point_coordinate(point)
    returned_value = []
    value = []

    for c in all_curves:
        pts = get_end_points(c)
        cpts1 = point_coordinate(pts[0])
        cpts2 = point_coordinate(pts[1])

        #   Nord
        if cpts1[1] > c_point[1] and cpts2[1] > c_point[1] and (
                cpts1[0] >= c_point[0] >= cpts2[0] or cpts2[0] >= c_point[0] >= cpts1[0]):
            value.append(1)

        #   Sud
        if cpts1[1] < c_point[1] and cpts2[1] < c_point[1] and (
                cpts1[0] >= c_point[0] >= cpts2[0] or cpts2[0] >= c_point[0] >= cpts1[0]):
            value.append(2)

        #   Est
        if cpts1[0] > c_point[0] and cpts2[0] > c_point[0] and (
                cpts1[1] >= c_point[1] >= cpts2[1] or cpts2[1] >= c_point[1] >= cpts1[1]):
            value.append(3)

        #   Ouest
        if cpts1[0] < c_point[0] and cpts2[0] < c_point[0] and (
                cpts1[1] >= c_point[1] >= cpts2[1] or cpts2[1] >= c_point[1] >= cpts1[1]):
            value.append(4)

    #   Suprimme les doublons

    for v in value:
        if not v in returned_value:
            returned_value.append(v)

    for c in all_curves:
        pts = get_end_points(c)
        if point == pts[0]:
            if get_dir(point, pts[1]) in returned_value:
                returned_value.remove(get_dir(point, pts[1]))
        elif point == pts[1]:
            if get_dir(point, pts[0]) in returned_value:
                returned_value.remove(get_dir(point, pts[0]))

    return returned_value


def old_can_cut(point, f_tag):
    # En utilisant le point et le f_tag, on cherche à voir s'il y a une curve adjaçant au point dans la même direction.
    """dir = 1 = nord | 2 = sud | 3 = est | 4 = ouest"""
    q = Query()
    cs = get_edge_tags()
    curves = []

    # On récupère tous les segments contenant le point utilisé
    for c in cs:
        ps = get_end_points(c)
        if point in ps:
            curves.append(c)

    directions = [1, 2, 3, 4]

    # Check si le coupage a déjà été fait
    for c in curves:
        point_to_check = None
        other_point = None
        for p in get_end_points(c):
            if p == point:
                point_to_check = point_coordinate(p)  # Coordonnées du point utilisé
            else:
                other_point = point_coordinate(p)  # Coordonnées de l'autre point du segment

        #
        # On retire directions dont les découpes qui ont déjà été faites
        #

        if point_to_check[1] < other_point[1]:
            if 1 in directions:
                directions.remove(1)
        if point_to_check[1] > other_point[1]:
            if 2 in directions:
                directions.remove(2)
        if point_to_check[0] < other_point[0]:
            if 3 in directions:
                directions.remove(3)
        if point_to_check[0] > other_point[0]:
            if 4 in directions:
                directions.remove(4)

    #
    # Avec les directions restante, on regarde lesquelles on peut garder
    #

    all_curves = get_edge_tags()
    c_point = point_coordinate(point)
    returned_value = []

    for c in all_curves:
        pts = get_end_points(c)
        cpts1 = point_coordinate(pts[0])
        cpts2 = point_coordinate(pts[1])

        if not (c_point == cpts1 or c_point == cpts2):

            for d in directions:
                if d == 1:
                    if not (c_point[1] < cpts1[1] and c_point[1] < cpts2[1]):
                        continue

                    if (cpts1[0] <= c_point[0] <= cpts2[0]) or (cpts1[0] >= c_point[0] >= cpts2[0]):
                        returned_value.append(1)


                if d == 2:
                    if not (c_point[1] > cpts1[1] and c_point[1] > cpts2[1]):
                        continue

                    if (cpts1[0] <= c_point[0] <= cpts2[0]) or (cpts1[0] >= c_point[0] >= cpts2[0]):
                        returned_value.append(2)

                if d == 3:
                    if not (c_point[0] < cpts1[0] and c_point[0] < cpts2[0]):
                        continue

                    if (cpts1[1] <= c_point[1] <= cpts2[1]) or (cpts1[1] >= c_point[1] >= cpts2[1]):
                        returned_value.append(3)

                if d == 4:
                    if not (c_point[0] > cpts1[0] and c_point[0] > cpts2[0]):
                        continue

                    if (cpts1[1] <= c_point[1] <= cpts2[1]) or (cpts1[1] >= c_point[1] >= cpts2[1]):
                        returned_value.append(4)

    return returned_value


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    """Probleme de récursion à régler"""

    # first, we initialize the gmsh environment
    initialize()
    # ======================================================
    # Create a first shape by assembling rectangles
    # ======================================================
    file_name = "shape_21.txt"
    f = open(file_name, "r")
    ps = create_shape(f)
    f.close()
    #print_infos()

    ## Cut from point 13, face 2 in direction 3 (east)
    # cut(13, 2, 3)
    # print_infos()
    ## Cut from point 3, face 1 in direction 2 (south)
    # cut(3, 1, 2)
    # print_infos()

    launch()
    print_infos()


    ## final meshing
    remesh()
    gmsh.write("mesh_gmsh.vtk")
    finalize()
