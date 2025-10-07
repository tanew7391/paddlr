

build:
	docker compose -f deployment/docker-compose.yml --env-file .env build	

up:
	docker compose -f deployment/docker-compose.yml --env-file .env up

up/debug:
	RUST_BACKTRACE=1 RUST_LOG=debug docker compose -f deployment/docker-compose.yml --env-file .env up


# Must be run on the first time after `make up` to load OSM data into PostGIS 
up/init:
#TODO: install postgis here if not exists
	docker cp prime_file.xml cartographer:/app/prime_file.xml
	docker exec -it cartographer sh -c "cat /app/prime_file.xml | osm2pgsql -d postgresql://${PGUSER}:${PGPASSWORD}@${PGHOST}:${PGPORT}/${PGDATABASE} --slim -r xml /dev/stdin"