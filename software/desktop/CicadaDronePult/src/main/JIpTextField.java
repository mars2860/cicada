package main;

/*
 *  Copyright (C) 2011 Naveed Quadri
 *  naveedmurtuza@gmail.com
 *  www.naveedmurtuza.blogspot.com
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

import java.awt.*;
import java.awt.event.*;
import java.io.Serializable;
import java.net.*;
import javax.swing.*;

/**
 * An IP Address COntrol
 * 
 * https://github.com/akuhtz/swing-components/blob/master/src/org/gpl/SwingComponents/JIPTextField/JIpTextField.java
 * 
 * @author Naveed Quadri
 */
public class JIpTextField extends JTextField implements Serializable {

    /**
	 * 
	 */
	private static final long serialVersionUID = 8405758004881964370L;

	/**
     * Enum for IP Version
     */
    public enum IPVersion {

        IPV4, IPV6
    }

    private enum Properties {

        BACKGROUND,
        FOREGROUND,
        ENABLED,
        FONT,
        SIZE
    }
    private final char IPV4_SEPARATOR = '.';
    private final char IPV6_SEPARATOR = ':';
    private final int IPV4_FIELDS = 4;
    private final int IPV6_FIELDS = 8;
    private int MAX_LEN = 3;
    private JTextField[] components;
    private char separator = IPV4_SEPARATOR;
    private IPVersion version;
    private boolean focus = false;
    private JTextField[] separatorComponents;

    /**
     * Constructs the IPV4 Field
     */
    public JIpTextField() {
        this(IPVersion.IPV4, null);
    }

    /**
     * Constructs the JIPTextField for the specified version
     * @param version IP Version
     */
    public JIpTextField(IPVersion version) {
        this(version, null);

    }

    /**
     * Constructs the JIPTextField for the specified version and IP Address
     * @param version IP Version
     * @param address IP Address
     */
    public JIpTextField(IPVersion version, InetAddress address) {
        this.version = version;
        initComponent();
        if (address != null) {
            setIpAddress(address);
        }
    }

    /**
     * Gets the IP Version
     * @return IP Version
     */
    public IPVersion getVersion() {
        return version;
    }

    /**
     * Sets the IP Version 
     * @param version IP Version
     */
    public void setVersion(IPVersion version) {
        this.version = version;
        removeAll();
        initComponent();
    }


    /**
     * Initializes the component
     */
    private void initComponent() {
        setPreferredSize(new Dimension(150, this.getPreferredSize().height));
        setLayout(new BoxLayout(this, BoxLayout.LINE_AXIS));
        if (version == IPVersion.IPV4) {
            separator = IPV4_SEPARATOR;
            MAX_LEN = 3;
        } else {
            separator = IPV6_SEPARATOR;
            MAX_LEN = 4;
        }
        components = getTextComponents();
        separatorComponents = getDotComponents(components.length - 1);

        for (int i = 0, j = 0; i
                < components.length; i++, j++) {
            add(components[i]);
            if (i != components.length - 1) {
                add(separatorComponents[j]);
            }
        }

    }

    /**
     * Gets the TextComponents for the control.
     * @return an array of 4 TextComponents if IPV4, 8 TextComponents if IPV6
     */
    private JTextField[] getTextComponents() {
        JTextField[] c = null;
        switch (version) {
            case IPV4:
                c = new JTextField[IPV4_FIELDS];
                break;
            case IPV6:
                c = new JTextField[IPV6_FIELDS];
        }
        for (int i = 0; i < c.length; i++) {
            c[i] = new TextComponent(this);
        }
        return c;
    }


    /**
     * Gets the Separator component
     * @param length will be 3 if IPV4, 7 if IPV6
     * @return read-only textfields with the separator. '.' for IPV4, ':' for IPV6
     */
    private JTextField[] getDotComponents(int length) {
        JTextField[] c = new JTextField[length];
        for (int i = 0; i < c.length; i++) {
            c[i] = getSeparatorComponent();
        }
        return c;
    }

    /**
     * Creates borderless, non-editable, non-focusable JTextfield component with the separator, '.' for IPV4, ':' for IPV6
     * @return JTextField
     */
    private JTextField getSeparatorComponent() {
        JTextField t = new JTextField();
        t.setFocusable(false);
        t.setEditable(false);
        t.setBorder(BorderFactory.createEmptyBorder());
        t.setBackground(getBackground());
        t.setForeground(getForeground());
        t.setText(String.valueOf(separator));
        t.setPreferredSize(new Dimension(5, 20));
        t.setSize(5, 20);
        t.setMaximumSize(new Dimension(5, 20));
        t.setMinimumSize(new Dimension(5, 20));
        return t;
    }

    private void setProperty(Properties prop, Object value) {
        if (components == null) {
            return;
        }
        for (JTextField textField : components) {
            switch (prop) {
                case BACKGROUND:
                    textField.setBackground((Color) value);
                    break;
                case FOREGROUND:
                    textField.setForeground((Color) value);
                    break;
                case ENABLED:
                    textField.setEnabled((Boolean) value);
                    break;
                case FONT:
                    textField.setFont((Font) value);
                    break;
                case SIZE:
                    textField.setPreferredSize((Dimension) value);
                    break;
            }
        }
        for (JTextField dotComponent : separatorComponents) {
            switch (prop) {
                case BACKGROUND:
                    dotComponent.setBackground((Color) value);
                    break;
                case FOREGROUND:
                    dotComponent.setForeground((Color) value);
                    break;
                case ENABLED:
                    dotComponent.setEnabled((Boolean) value);
                    break;
                case FONT:
                    dotComponent.setFont((Font) value);
                    break;
                case SIZE:
                    dotComponent.setPreferredSize(new Dimension(5, ((Dimension) value).height));
                    break;

            }
        }
    }

    /**
     * Gets an array of integers containing each field of the control
     * @return an array of integers containing each field of the control
     */
    public int[] getOctects() {
        int[] octects = new int[separator == IPV4_SEPARATOR ? IPV4_FIELDS : IPV6_FIELDS];
        for (int i = 0; i < octects.length; i++) {
            octects[i] = Integer.parseInt((components[i]).getText(), separator == IPV4_SEPARATOR ? 10 : 16);
        }
        return octects;
    }

    /**
     * Gets the IP Address
     * @return IP Address
     * @throws UnknownHostException
     */
    public InetAddress getIpAddress() throws UnknownHostException {
        return Inet4Address.getByName(getIpAddressString());
    }

    /**
     * Gets the ip address as string
     * @return IP Address String
     */
    public String getIpAddressString() {
        StringBuilder sb = new StringBuilder();
        for (JTextField component : components) {
            sb.append((component).getText());
            sb.append(separator);
        }
        //remove the last separtor
        sb.deleteCharAt(sb.length() - 1);
        return sb.toString();
    }

    /**
     * Sets theip address
     * @param ipAddress
     */
    public void setIpAddress(InetAddress ipAddress) {
        if (ipAddress instanceof Inet4Address && version != IPVersion.IPV4) {
            throw new IllegalArgumentException("The IPVersion of this component is IPV6 \n where as the IP Address passed in is IPV4");
        }
        if (ipAddress instanceof Inet6Address && version != IPVersion.IPV6) {
            throw new IllegalArgumentException("The IPVersion of this component is IPV4 \n where as the IP Address passed in is IPV6");
        }
        String ip = ipAddress.getHostAddress();
        String regex = separator == IPV4_SEPARATOR ? "\\" : "";
        regex += String.valueOf(separator);
        String[] octects = ip.split(regex);
        for (int i = 0; i < octects.length; i++) {
            (components[i]).setText(octects[i]);
        }
    }

    // <editor-fold defaultstate="collapsed" desc="Overrides">
    @Override
    public boolean hasFocus() {
        return focus;
    }

    @Override
    public void grabFocus() {
        if (components != null) {
            components[0].grabFocus();
        }
    }

    @Override
    public void setBackground(Color bg) {
        super.setBackground(bg);
        setProperty(Properties.BACKGROUND, bg);
    }

    @Override
    public void setEnabled(boolean enabled) {
        super.setEnabled(enabled);
        setProperty(Properties.ENABLED, enabled);
    }

    @Override
    public void setFont(Font font) {
        super.setFont(font);
        setProperty(Properties.FONT, font);
    }

    @Override
    public void setForeground(Color fg) {
        super.setForeground(fg);
        setProperty(Properties.FOREGROUND, fg);
    }

    @Override
    public void setPreferredSize(Dimension preferredSize) {
        super.setPreferredSize(preferredSize);
        setProperty(Properties.SIZE, preferredSize);
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Color old = g.getColor();
        g.setColor(getBackground());
        g.fillRect(getX(),
                getY(), getSize().width,
                getSize().height);
        g.setColor(old);
    }

    @Override
    public String getText() {
        return getIpAddressString();
    }

    @Override
    public void setText(String t) {

        try {

            InetAddress ip = version == IPVersion.IPV4 ? Inet4Address.getByName(t) : Inet6Address.getByName(t);
            setIpAddress(ip);
        } catch (UnknownHostException ex) {
            JOptionPane.showMessageDialog(this, "Not a valid value for IP Address", "Information", JOptionPane.INFORMATION_MESSAGE);
        }
    }// </editor-fold>

    /**
     *
     */
    class TextComponent extends JTextField implements ActionListener, FocusListener, KeyListener {

        /**
		 * 
		 */
		private static final long serialVersionUID = 737631169836578865L;
		
		private JIpTextField ref;

        private TextComponent(JIpTextField ref) {
            init();
            this.ref = ref;
        }

        private void init() {
            setBorder(BorderFactory.createEmptyBorder());
            setHorizontalAlignment(CENTER);
            addActionListener(this);
            addFocusListener(this);
            addKeyListener(this);
            setInputVerifier(version == IPVersion.IPV4 ? new IPV4Verifier() : new IPV6Verifier());
        }

        public void actionPerformed(ActionEvent e) {
            transferFocus();
        }

        public void focusGained(FocusEvent e) {
            selectText(((JTextField) e.getComponent()));
            focus = true;
            ref.repaint();
        }

        public void focusLost(FocusEvent e) {
            focus = false;
            ref.repaint();
        }

        public void keyTyped(KeyEvent e) {
            JTextField source = (JTextField) e.getSource();
            if (e.getKeyChar() == separator) {
                e.setKeyChar('\0');
                if (!source.getText().isEmpty()) {
                    transferFocus();
                }
            }
            if (source.getText().isEmpty() && e.getKeyChar() == KeyEvent.VK_BACK_SPACE) {
                transferFocusBackward();
            }
            switch (version) {
                case IPV4:
                    if (!Character.isDigit(e.getKeyChar())) {
                        e.setKeyChar('\0');
                    }
                    break;
                case IPV6:
                    try {
                        Integer.parseInt(String.valueOf(e.getKeyChar()), 16);
                    } catch (Exception ex) {
                        e.setKeyChar('\0');
                    }
                    break;
            }
        }

        public void keyPressed(KeyEvent e) {
        }

        public void keyReleased(KeyEvent e) {
            if (((JTextField) e.getSource()).getText().length() == MAX_LEN) {
                transferFocus();
            }
        }

        private void selectText(JTextField jTextField) {
            String text = jTextField.getText();
            if (text.isEmpty()) {
                return;

            }
            jTextField.setSelectionStart(0);
            jTextField.setSelectionEnd(text.length());
        }
    }

    // <editor-fold defaultstate="collapsed" desc="InputVerifiers">
    class IPV6Verifier extends InputVerifier {

        @Override
        public boolean verify(JComponent input) {
            JTextField inputTxt = (JTextField) input;
            //allow empty octects
            if (inputTxt.getText().isEmpty()) {
                return true;
            }
            try {
                /*int value = */Integer.parseInt(inputTxt.getText(), 16);
                inputTxt.setText(inputTxt.getText().toUpperCase());
                return true;
            } catch (Exception ex) {
                return false;
            }
        }
    }

    class IPV4Verifier extends InputVerifier {

        @Override
        public boolean verify(JComponent input) {
            JTextField inputTxt = (JTextField) input;
            //allow empty octects
            if (inputTxt.getText().isEmpty()) {
                return true;
            }
            try {
                int value = Integer.parseInt(inputTxt.getText());
                if (value > 255) {
                    value = 255;
                }
                inputTxt.setText("" + value);
                return true;
            } catch (Exception ex) {
                return false;
            }
        }
    }
    //</editor-fold>
}