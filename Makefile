all: examples README.md
examples: avr-cmake.org
	time emacs --batch --no-init-file --load .org-mode.emacs.el --find-file avr-cmake.org --funcall org-babel-tangle --kill
README.md: avr-cmake.org
	time emacs --batch --load .org-mode.emacs.el --find-file avr-cmake.org --eval '(export-gfm "README.md")' --kill
clean:
	-rm -rf README.md examples
