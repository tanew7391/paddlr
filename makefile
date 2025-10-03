

build:
	docker compose -f deployment/docker-compose.yml --env-file .env build	

up:
	docker compose -f deployment/docker-compose.yml --env-file .env up

up/debug:
	RUST_LOG=debug docker compose -f deployment/docker-compose.yml --env-file .env up
