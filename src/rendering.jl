# Used for rendering
using PyCall
@pyimport pyglet
@pyimport sys
@pyimport os
@pyimport pyglet.gl as pygl
@pyimport numpy as np

@pyimport math
#=
type viewer
    width::Int
    height::Int
    window::Any
    isopen::Bool
    geoms::Array{Any,1}
    onetime_geoms::Array{Any,1}
    transform::Any
end
=#
RAD2DEG = 57.29577951308232 # A magical number
function _add_attrs(geom, attrs)
    if "color" in keys(attrs)
        geom[:set_color](attrs["color"][1], attrs["color"][2], attrs["color"][3])
    end
    if "linewidth" in keys(attrs)
        geom[:set_linewidth](attrs["linewidth"])
    end
end

@pydef mutable struct Viewer
    function __init__(self, width::Int, height::Int, x=0, y=0, display=nothing)
        self[:width] = width
        self[:height] = height
        self[:window] = pyglet.window[:Window](width=width, height=height, display=display)
        self[:window][:on_close] = self[:window_closed_by_user]
        self[:window][:set_location](x, y)
        self[:isopen] = true
        self[:geoms] = Any[]
        self[:onetime_geoms] = Any[]
        self[:transform] = Transform()
        pygl.glEnable(pygl.GL_BLEND)
        pygl.glBlendFunc(pygl.GL_SRC_ALPHA, pygl.GL_ONE_MINUS_SRC_ALPHA)
    end

    function close(self)
        self[:window][:close]()
    end

    function window_closed_by_user(self)
        self[:isopen] = false
    end
    function set_bounds(self, left, right, bottom, top)
        assert(right > left && top > bottom)
        scalex = self[:width]/(right-left)
        scaley = self[:height]/(top-bottom)
        self[:transform] = Transform(translation=(-left*scalex, -bottom*scaley), scale=(scalex, scaley))
    end

    function add_geom(self, geom)
        self[:geoms] = push!(self[:geoms], geom)
    end
    function add_onetime(self, geom)
        self[:onetime_geoms] = push!(self[:onetime_geoms], geom)
    end

    function render(self, return_rgb_array=false)
        pygl.glClearColor(1,1,1,1)

        self[:window][:clear]()

        self[:window][:switch_to]()
        self[:window][:dispatch_events]()

        self[:transform][:enable]()

        for geom in self[:geoms]
            geom[:render]()
        end
        for geom in self[:onetime_geoms]
            geom[:render]()
        end
        self[:transform][:disable]()

        if return_rgb_array
            buffer = pyglet.image[:get_buffer_manager]()[:get_color_buffer]()
            image_data = buffer[:get_image_data]()
        end
        self[:window][:flip]()
        self[:onetime_geoms] = Any[]

        return return_rgb_array?image_data:self[:isopen]
    end
    # Convenience
    function draw_circle(self, attrs, radius=10, res=30, filled=true)
        geom = make_circle(radius=radius, res=res, filled=filled)
        _add_attrs(geom, attrs)
        self[:add_onetime](geom)
        return geom
    end
    function draw_polygon(self, v, attrs, filled=true)
        geom = make_polygon(v=v, filled=filled)
        _add_attrs(geom, attrs)
        self[:add_onetime](geom)
        return geom
    end
    function draw_polyline(self, v, attrs)
        geom = make_polyline(v=v)
        _add_attrs(geom, attrs)
        self[:add_onetime](geom)
        return geom
    end
    function draw_line(self, start, dest, attrs)
        geom = Line(start, dest)
        _add_attrs(geom, attrs)
        self[:add_onetime](geom)
        return geom
    end
    function get_array(self)
        self[:window][:flip]()
        image_data = pyglet.image[:get_buffer_manager]()[:get_color_buffer]()[:get_image_data]()
        self[:window][:flip]()
        arr = np.fromstring(image_data[:data], dtype=np.uint8, sep="")
        arr = arr[:reshape](self[:height], self[:width], 4)
        return arr[end:-1:1,:,1:3]
    end
    function __del__(self)
        self[:close]()
    end
end
@pydef mutable struct Geom
    function __init__(self)
        self[:_color] = Color((0,0,0,1.0))
        self[:attrs] = [self[:_color]]
    end
    function render(self)
        for attr in self[:attrs][end:-1:1]
            attr[:enable]()
        end
        self[:render1]()
        for attr in self[:attrs]
            attr[:disable]()
        end
    end
    function render1(self)
        throw("unimplemented")
    end
    function  add_attr(self, attr)
        self[:attrs] = push!(self[:attrs], attr)
    end
    function set_color(self, r, g, b)
        self[:_color][:vec4] = (r, g, b, 1)
    end
end
@pydef mutable struct Attr
    function enable(self)
        throw("enable unimplemented")
    end
    function disable(self)
        nothing
    end
end

@pydef mutable struct Transform <: Attr
    function __init__(self; translation=(0.0, 0.0), rotation=0.0, scale=(1,1))
        self[:set_translation](translation[1], translation[2])
        self[:set_rotation](rotation)
        self[:set_scale](scale[1], scale[2])
    end
    function enable(self)
        pygl.glPushMatrix()
        pygl.glTranslatef(self[:translation][1], self[:translation][2], 0) # translate to GL loc ppint
        pygl.glRotatef(RAD2DEG * self[:rotation], 0, 0, 1.0)
        pygl.glScalef(self[:scale][1], self[:scale][2], 1)
    end
    function disable(self)
        pygl.glPopMatrix()
    end
    function set_translation(self, newx, newy)
        self[:translation] = (float(newx), float(newy))
    end
    function set_rotation(self, new_rot)
        self[:rotation] = float(new_rot)
    end
    function set_scale(self, newx, newy)
        self[:scale] = (float(newx), float(newy))
    end
end

@pydef mutable struct Color <: Attr
    function __init__(self, vec4)
        self[:vec4] = vec4
    end
    function enable(self)
        pygl.glColor4f(self[:vec4][1],self[:vec4][2],self[:vec4][3],self[:vec4][4])
    end
end

@pydef mutable struct LineStyle <: Attr
    function __init__(self, style)
        self[:style] = style
    end
    function enable(self)
        pygl.glEnable(pygl.GL_LINE_STIPPLE)
        pygl.glLineStipple(1, self[:style])
    end
    function disable(self)
        pygl.glDisable(pygl.GL_LINE_STIPPLE)
    end
end

@pydef mutable struct LineWidth <: Attr
    function __init__(self, stroke)
        self[:stroke] = stroke
    end
    function enable(self)
        pygl.glLineWidth(self[:stroke])
    end
end

@pydef mutable struct Point <:Geom
    function __init__(self)
        Geom[:__init__](self)
    end
    function render1(self)
        pygl.glBegin(pygl.GL_POINTS) # draw point
        pygl.glVertex3f(0.0, 0.0, 0.0)
        pygl.glEnd()
    end
end
@pydef mutable struct FilledPolygon <: Geom
    function __init__(self, v)
        Geom[:__init__](self)
        self[:v] = v
    end
    function render1(self)
        if   length(self[:v]) == 4
            pygl.glBegin(pygl.GL_QUADS)
        elseif length(self[:v])  > 4
            pygl.glBegin(pygl.GL_POLYGON)
        else
            pygl.glBegin(pygl.GL_TRIANGLES)
        end
        for p in self[:v]
            pygl.glVertex3f(p[1], p[2],0)  # draw each vertex
        end
        pygl.glEnd()
    end
end

function make_circle(radius=10, res=30, filled=true)
    points = []
    for i = 1:res
        ang = 2*math.pi*(i-1) / res
        push!(points, (math.cos(ang)*radius, math.sin(ang)*radius))
    end
    if filled
        return FilledPolygon(points)
    else
        return PolyLine(points, true)
    end
end

function make_polygon(v, filled=true)
    if filled
        return FilledPolygon(v)
    else
        return PolyLine(v, true)
    end
end
function make_polyline(v)
    return PolyLine(v, false)
end

function make_capsule(length, width)
    l, r, t, b = 0, length, width/2, -width/2
    box = make_polygon([(l,b), (l,t), (r,t), (r,b)])
    circ0 = make_circle(width/2)
    circ1 = make_circle(width/2)
    circ1[:add_attr](Transform(translation=(length, 0)))
    geom = Compound([box, circ0, circ1])
    return geom
end

@pydef mutable struct Compound <: Geom
    function __init__(self, gs)
        Geom[:__init__](self)
        self[:gs] = gs
        for g in self[:gs]
            g[:attrs] = [a for a in g[:attrs] if !py"isinstance($a, $Color)"]
        end
    end
    function add_geom(self, g)
        self[:gs] = push!(self[:gs], g)
    end
    function render1(self)
        for g in self[:gs]
            g[:render]()
        end
    end
end
@pydef mutable struct Text <: Geom
    function __init__(self,text, x, y, font_size=11, font_name="Times New Roman")
        Geom[:__init__](self)
        self[:attrs] = []
        self[:label] = pyglet.text[:Label](text,
                                    font_name=font_name,
                                    font_size=font_size,
                                    x=x, y=y,
                                    color=(0, 0, 0, 255),
                                    anchor_x="center", anchor_y="center")
    end
    function render1(self)
        self[:label][:draw]()
    end
end
@pydef mutable struct BarPlot <: Geom
    function __init__(self, width, height, x, y)
        Geom[:__init__](self)
        self[:x] = x
        self[:y] = y
        self[:width] = width
        self[:height] = height
        xaxis = Line((x, y), (x+width, y))
        yaxis = Line((x, y), (x, y+height))
        xarrow = FilledPolygon([(x+width+(5*sqrt(3)), y), (x+width, y+5), (x+width, y-5)])
        yarrow = FilledPolygon([(x, y+height+(5*sqrt(3))), (x-5, y+height), (x+5, y+height)])
        xaxis[:set_linewidth](1.5)
        yaxis[:set_linewidth](1.5)
        #xaxis[:set_color](1,0,0)
        #yaxis[:set_color](1,0,0)
        #xarrow[:set_color](1,0,0)
        #yarrow[:set_color](1,0,0)
        self[:base] = [xarrow,xaxis,yarrow,yaxis]

        self[:bars] = Compound([])
        self[:maxcount] = 50
    end
    function set_axis(self, texts)
        xlabel = Text(texts[1], self[:x]+self[:width]/2, self[:y]-10)
        ylabel = Text(texts[2], 0,0)
        ylabel[:add_attr](Transform(translation=(self[:x]-10, self[:y]+self[:height]/2), rotation=pi/2))
        self[:base] = push!(self[:base], xlabel)
        self[:base] = push!(self[:base], ylabel)
    end
    function add_data(self, data; color=nothing, renew=false)
        maxcount = -1
        for group in data
            for elem in group
                if elem > maxcount
                     maxcount=elem
                end
            end
        end
        self[:maxcount] = Int(ceil(maxcount/50)*50)
        k = 4
        margin = self[:width]/((k+1)*length(data)+1)
        barwidth = k*margin
        if renew
            self[:bars][:gs] = []
            for (idx,group) in enumerate(data)
                c = color[idx]
                if length(group) == 1
                    bar = make_polygon(((self[:x]+margin*idx+barwidth*(idx-1), self[:y]+0.75),
                                        (self[:x]+margin*idx+barwidth*idx, self[:y]+0.75),
                                        (self[:x]+margin*idx+barwidth*idx, self[:y]+0.75+(group[1]/self[:maxcount])*self[:height]),
                                        (self[:x]+margin*idx+barwidth*(idx-1), self[:y]+0.75+(group[1]/self[:maxcount])*self[:height])), true)
                    bar[:set_color](c[1], c[2], c[3])
                    self[:bars][:add_geom](bar)
                elseif length(group) == 2
                    bar1 = make_polygon(((self[:x]+margin*idx+barwidth*(idx-1), self[:y]+0.75),
                                        (self[:x]+margin*idx+barwidth*(idx-1)+(barwidth/2-1.0), self[:y]+0.75),
                                        (self[:x]+margin*idx+barwidth*(idx-1)+(barwidth/2-1.0), self[:y]+0.75+(group[1]/self[:maxcount])*self[:height]),
                                        (self[:x]+margin*idx+barwidth*(idx-1), self[:y]+0.75+(group[1]/self[:maxcount])*self[:height])), true)
                    bar1[:set_color](c[1], c[2], c[3])
                    bar2 = make_polygon(((self[:x]+margin*idx+barwidth*(idx-1)+(barwidth/2+1.0), self[:y]+0.75),
                                        (self[:x]+margin*idx+barwidth*(idx-1)+barwidth, self[:y]+0.75),
                                        (self[:x]+margin*idx+barwidth*(idx-1)+barwidth, self[:y]+0.75+(group[2]/self[:maxcount])*self[:height]),
                                        (self[:x]+margin*idx+barwidth*(idx-1)+(barwidth/2+1.0), self[:y]+0.75+(group[2]/self[:maxcount])*self[:height])), false)
                    bar2[:set_color](c[1], c[2], c[3])
                    self[:bars][:add_geom](bar1)
                    self[:bars][:add_geom](bar2)
                else
                    throw("unimplemented")
                end
            end
            #println(length(self[:bars][:gs]))
        else
            for (idx,group) in enumerate(data)
                if length(group) == 1
                    self[:bars][:gs][idx][:v] = ((self[:x]+margin*idx+barwidth*(idx-1), self[:y]+0.75),
                                                 (self[:x]+margin*idx+barwidth*idx, self[:y]+0.75),
                                                 (self[:x]+margin*idx+barwidth*idx, self[:y]+0.75+(group[1]/self[:maxcount])*self[:height]),
                                                 (self[:x]+margin*idx+barwidth*(idx-1), self[:y]+0.75+(group[1]/self[:maxcount])*self[:height]))
                elseif length(group) == 2
                    #println(length(self[:bars][:gs]))
                    #println((idx-1)*2+1)
                    self[:bars][:gs][(idx-1)*2+1][:v] = ((self[:x]+margin*idx+barwidth*(idx-1), self[:y]+0.75),
                                                         (self[:x]+margin*idx+barwidth*(idx-1)+(barwidth/2-1.0), self[:y]+0.75),
                                                         (self[:x]+margin*idx+barwidth*(idx-1)+(barwidth/2-1.0), self[:y]+0.75+(group[1]/self[:maxcount])*self[:height]),
                                                         (self[:x]+margin*idx+barwidth*(idx-1), self[:y]+0.75+(group[1]/self[:maxcount])*self[:height]))
                    self[:bars][:gs][(idx-1)*2+2][:v] = ((self[:x]+margin*idx+barwidth*(idx-1)+(barwidth/2+1.0), self[:y]+0.75),
                                                         (self[:x]+margin*idx+barwidth*(idx-1)+barwidth, self[:y]+0.75),
                                                         (self[:x]+margin*idx+barwidth*(idx-1)+barwidth, self[:y]+0.75+(group[2]/self[:maxcount])*self[:height]),
                                                         (self[:x]+margin*idx+barwidth*(idx-1)+(barwidth/2+1.0), self[:y]+0.75+(group[2]/self[:maxcount])*self[:height]))
                else
                    throw("unimplemented")
                end
            end
        end
    end
    function add_bars(self, bar)
        self[:bars] = push!(self[:bars], bar)
    end
    function render1(self)
        self[:bars][:render]()
        for g in self[:base]
            g[:render]()
        end
    end
end
@pydef mutable struct PolyLine <:Geom
    function __init__(self, v, close)
        Geom[:__init__](self)
        self[:v] = v
        self[:close] = close
        self[:linewidth] = LineWidth(1)
        self[:add_attr](self[:linewidth])
    end
    function render1(self)
        pygl.glBegin(self[:close] ? pygl.GL_LINE_LOOP : pygl.GL_LINE_STRIP)
        for p in self[:v]
            pygl.glVertex3f(p[1], p[2],0)  # draw each vertex
        end
        pygl.glEnd()
    end
    function set_linewidth(self, x)
        self[:linewidth][:stroke] = x
    end
end

@pydef mutable struct Line <: Geom
    function __init__(self, start=(0.0, 0.0), dest=(0.0, 0.0))
        Geom[:__init__](self)
        self[:start] = start
        self[:dest] = dest
        self[:linewidth] = LineWidth(1)
        self[:add_attr](self[:linewidth])
    end

    function render1(self)
        pygl.glBegin(pygl.GL_LINES)
        pygl.glVertex2f(self[:start][1], self[:start][2])
        pygl.glVertex2f(self[:dest][1], self[:dest][2])
        pygl.glEnd()
    end
    function set_linewidth(self, x)
        self[:linewidth][:stroke] = x
    end
end

@pydef mutable struct Image <: Geom
    function __init__(self, fname, width, height,x=0,y=0)
        Geom[:__init__](self)
        self[:width] = width
        self[:height] = height
        img = pyglet.image[:load](fname, decoder=pyglet.image[:codecs][:png][:PNGImageDecoder]())
        self[:img] = img
        self[:attrs] = []
        self[:flip] = false
        self[:x] = x
        self[:y] = y
        self[:isshown] = true
    end
    function render1(self)
        #self[:texture][:blit](-self[:width]/2, 0, width=self[:width], height=self[:height])
        #self[:texture][:blit](-self[:width]/2, 0, width=self[:width], height=self[:height])
        #self[:img][:blit](-self[:width]/2, -self[:height]/2, width=self[:width], height=self[:height])
        if self[:isshown]
            self[:img][:blit](self[:x],self[:y], width=self[:width], height=self[:height])
        end
    end
end

@pydef mutable struct ImageCompound <: Geom
    function __init__(self, fnames, width, height, handler)
        Geom[:__init__](self)
        self[:width] = width
        self[:height] = height
        self[:imgs] = []
        for f in fnames
            self[:imgs]=push!(self[:imgs], pyglet.image[:load](f, decoder=pyglet.image[:codecs][:png][:PNGImageDecoder]()))
        end
        #self[:xs] = xs
        #self[:ys] = ys
        #self[:cidxs] = cidxs
        self[:attrs] = []
        self[:flip] = false
        self[:handler] = handler
        #self[:isshowns] = ones(Int, length(xs))
    end
    function render1(self)
        handler=self[:handler]
        for (idx, value) in enumerate(handler.isshowns)
            if value == 1
                cidx = handler.cidxs[idx]
                x = handler.xs[idx]
                y = handler.ys[idx]
                self[:imgs][cidx][:blit](x,y, width=self[:width], height=self[:height])
            end
        end
    end

end
#=
function main()
    screen_width = 600
    screen_height = 400
    viewer = Viewer(screen_width, screen_height)
    cartwidth = 50.0
    cartheight = 30.0
    l,r,t,b = -cartwidth/2, cartwidth/2, cartheight/2, -cartheight/2
    cart = FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
    carttrans = Transform()
    cart[:add_attr](carttrans)
    viewer[:add_geom](cart)
    x_threshold = 2.4
    world_width = x_threshold*2
    scale = screen_width/world_width
    carty = 100 # TOP OF CART
    polewidth = 10.0
    polelen = scale * 1.0
    l,r,t,b = -polewidth/2,polewidth/2,polelen-polewidth/2,-polewidth/2
    pole = FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
    pole[:set_color](.8,.6,.4)
    axleoffset =cartheight/4.0
    poletrans = Transform(translation=(0, axleoffset))
    pole[:add_attr](poletrans)
    pole[:add_attr](carttrans)
    viewer[:add_geom](pole)
    axle = make_circle(polewidth/2)
    axle[:add_attr](poletrans)
    axle[:add_attr](carttrans)
    axle[:set_color](.5,.5,.8)
    viewer[:add_geom](axle)
    track = Line((0,carty), (screen_width,carty))
    track[:set_color](0,0,0)
    viewer[:add_geom](track)
    for i  = 1:1000
        x = [-0.01258752,  0.02973232, -0.04936275, -0.00062176]
        x = rand(-0.05:0.01:0.05, 4, 1)
        cartx = x[1]*scale + screen_width/2.0 # MIDDLE OF CART
        carttrans[:set_translation](cartx, carty)
        poletrans[:set_rotation](-x[3])
        viewer[:render]()
    end
    viewer[:close]()
end
=#
