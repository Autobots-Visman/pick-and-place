.PHONY: build up down up-%

build:
	docker compose build

up:
	docker compose up

down:
	docker compose down --remove-orphans

# also make target for an override
up-%:
	docker compose -f docker-compose.yml -f docker-compose.$*.yml up
