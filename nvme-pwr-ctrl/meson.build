executable(
    'nvme_pwr_ctrl',
    [
        'nvme_pwr_ctrl.cpp',
    ],
    dependencies: [
        dependency('boost'),
        dependency('sdbusplus'),
        dependency('libgpiod'),
        dependency('libgpiodcxx'),
        dependency('threads'),
    ],
    install: true,
    install_dir: get_option('bindir')
)

configure_file(
    copy: true,
    input: 'nvme_power_gpio.json',
    install: true,
    install_dir: '/etc/nvme',
    output: 'nvme_power_gpio.json'
)

systemd = dependency('systemd')
conf_data = configuration_data()
conf_data.set('bindir', get_option('prefix') / get_option('bindir'))
configure_file(
  input: 'xyz.openbmc_project.Control.Nvme.Power.service.in',
  output: 'xyz.openbmc_project.Control.Nvme.Power.service',
  configuration: conf_data,
  install: true,
  install_dir: systemd.get_pkgconfig_variable('systemdsystemunitdir'))