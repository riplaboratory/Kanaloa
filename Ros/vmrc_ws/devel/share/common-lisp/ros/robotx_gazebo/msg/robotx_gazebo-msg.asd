
(cl:in-package :asdf)

(defsystem "robotx_gazebo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "UsvDrive" :depends-on ("_package_UsvDrive"))
    (:file "_package_UsvDrive" :depends-on ("_package"))
  ))