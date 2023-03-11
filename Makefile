all: deploy

deploy:
	rp ./gradlew deploy && git tag -f onbot


push: 
	rp git push --tags -f
	rp git push


.PHONY: build
build:
	./gradlew build
