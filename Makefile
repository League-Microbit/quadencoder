all: deploy

build:
	pxt build

deploy:
	pxt deploy

test: 
	pxt build && pxt deploy && pxt test
