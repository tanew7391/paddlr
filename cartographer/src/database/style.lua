local water_table = nil

water_table = osm2pgsql.define_way_table('water', {
    {
        column = 'geometry',
        type = 'geometry',
        not_null = true,
        projection = 4326
    }, {
        column = 'name',
        type = 'text'
    }, {
        column = 'type',
        type = 'text'
    } -- 'way' or 'relation'
    
})

-- Process ways with natural=water
function osm2pgsql.process_way(object)
    if object.tags["natural"] == "water" and object.is_closed then
        water_table:insert({
            osm_id = object.id,
            geometry = object:as_polygon(),
            name = object.tags["name"],
            type = 'way'
        })
    end
end

-- Process relations with natural=water
function osm2pgsql.process_relation(object)
    if object.tags["natural"] == "water" then
        local geom = object:as_multipolygon()
        if geom then
            water_table:insert({
                osm_id = object.id,
                geometry = geom,
                name = object.tags["name"],
                type = 'relation'
            })
        end
    end
end

