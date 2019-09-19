
(cl:in-package :asdf)

(defsystem "way_point_wamv-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "add_way_point" :depends-on ("_package_add_way_point"))
    (:file "_package_add_way_point" :depends-on ("_package"))
  ))