require_relative 'model'
Record.where(version: 30).delete_all
