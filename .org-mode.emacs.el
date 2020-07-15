;; Config to build the notebook

(setq package-enable-at-startup nil)
(package-initialize)

(require 'org)
(require 'ob-tangle)
(require 'ox-org)
(require 'ox-gfm)

(org-babel-do-load-languages
 'org-babel-load-languages
 '((shell . t)
   (C . t)))

(defun export-gfm (output-path)
  (org-gfm-export-as-markdown nil nil nil)
  (set-visited-file-name output-path)
  (save-buffer))
