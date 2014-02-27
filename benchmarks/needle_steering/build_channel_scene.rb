require 'builder'
require 'trollop'



#DATA = {
#  implant_cylinder_radius: 1.75,
#  implant_cylinder_height: 7,
#  implant_entry_length: 1,
#  implant_entry_radius: 0.15,
#  vertical_surround_radius: 1.4
#}

OPTS = Trollop::options do
  opt :type, "Type of output", :type => :string
  opt :debug, "Debug", :type => :boolean, :default => false
end

height = HEIGHT = 7
radius = RADIUS = 2.5
target_height = TARGET_HEIGHT = 1
target_radius = TARGET_RADIUS = 0.15
NEEDLES = []

rotmap = {
  "0 0 0 0" => "1 0 0 90",
  "0 0 1 90" => "0 1 0 90",
  "1 0 0 90" => "0 1 0 90",
  #"0 0 1 0" => "0 1 0 -90",
}

def needle_to_joints(needle)
  rots = needle[3].split.map(&:to_f)
  x, y, z = rots[0..2]; size = rots[3] * Math::PI / 180
  x *= size; y *= size; z *= size
  [needle[0], needle[1], needle[2], x, y, z]

end

def build_implant(b, translation_x, translation_y, translation_z, rotationaxis="", needleaxis="")
  if OPTS[:debug]
    b.Geom(type: "cylinder") do |g|
      g.radius TARGET_RADIUS
      g.height TARGET_HEIGHT
      g.translation "#{translation_x} #{translation_y} #{translation_z}"
      if rotationaxis.size > 0
        g.rotationaxis rotationaxis
      end
    end
  end
  NEEDLES << [translation_x, translation_y, translation_z, needleaxis]#(if rotationaxis.size > 0 then rotationaxis else "0 0 0 0" end)]
end

xml = Builder::XmlMarkup.new( :indent => 2 )
xml.instruct! :xml, :encoding => "ASCII"

goal_joints = []
xml.Environment do |e|
  e.KinBody(name: "ImplantKinBody") do |kb|
    kb.Body(name: "ImplantBody") do |b|
      b.Geom(type: "sphere") do |g|
        g.radius radius
        g.translation "0 0 #{height}"
        g.transparency 0.1
      end
      b.Geom(type: "cylinder") do |g|
        g.radius radius
        g.height height
        g.rotationaxis "1 0 0 90"
        g.translation "0 0 #{height*0.5}"
        g.transparency 0.1
      end
      # build target points on top
      angle = 67.5
      [[-1, 0, "1 0 0 -#{angle}"], [1, 0, "1 0 0 #{angle}"], [0, -1, "0 1 0 -#{angle}"], [0, 1, "0 1 0 #{angle}"]].each do |cx, cy, needleaxis|
        rotationaxis = if cy != 0 then "0 0 1 90" else "" end
        build_implant(b, cx*radius*0.5, cy*radius*0.5, height, rotationaxis, needleaxis)
      end
      # build vertical target points surrounding the inside of cylinder
      -135.step(-225, -30).each do |deg|
        rad = deg * Math::PI / 180
        build_implant(b, Math::cos(rad)*2.2, Math::sin(rad)*2.2, height*0.5, "1 0 0 90", "1 0 0 0")
      end
      45.step(-45, -45).each do |deg|
        rad = deg * Math::PI / 180
        build_implant(b, Math::cos(rad)*2, Math::sin(rad)*2, height*0.5, "0 -1 1 90", "0 1 0 22.5")
      end
    end
  end
  cnt = 0
  NEEDLES.each do |needle|
    cnt += 1
    #e.Robot(name: "needlebot_#{cnt}", file: "needlebot.xml") do |r|
    #  #puts needle.inspect
    #  #rots = needle[3].split.map(&:to_f)
    #  #x, y, z = rots[0..3]
    #  #size = rots[3]*Math::PI/180
    #  #x *= size; y *= size; z *= size
    #  r.translation needle[0..2].join(" ")#"#{needlejointvalues [needle[0], needle[1], needle[2], x, y, z].join(" ")
    #  r.rotationaxis needle[3]
    #end
    goal_joints << needle_to_joints(needle)
  end
end

case OPTS[:type]
when "xml"
  puts xml.target!
when "poses"
  goal_joints.each do |goal_joint|
    puts %Q{start_string_vec.push_back("#{goal_joint[0..1].join(",")},0,0,0,0");}
    puts %Q{goal_string_vec.push_back("#{goal_joint.join(",")}");}
  end
when "pose_data"
  puts "starts = ["
  goal_joints.each do |goal_joint|
    puts %Q{[#{goal_joint[0..1].join(",")},0,0,0,0],}
  end
  puts "]"
  puts "goals = ["
  goal_joints.each do |goal_joint|
    puts %Q{[#{goal_joint.join(",")}]}
  end
  puts "]"
end
